#!/usr/bin/env python3

import os
import json
import sys
import threading
from pathlib import Path
from queue import Empty, Full, Queue

import cv2
import message_filters
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from tracking_detector.msg import BoundingBox, BoundingBoxes


class YoloeSemanticPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.output_width = int(rospy.get_param("~output_width", 640))
        self.output_height = int(rospy.get_param("~output_height", 480))
        self.confidence = float(rospy.get_param("~confidence", 0.6))
        self.debug = bool(rospy.get_param("~debug", False))
        self.publish_visualization = bool(rospy.get_param("~publish_visualization", True))
        self.odom_sync_slop = max(1e-9, float(rospy.get_param("~odom_sync_slop", 0.05)))
        self.yoloe_root = self._resolve_yoloe_root()
        self._load_model()

        bounding_boxes_topic = rospy.get_param("~bounding_boxes_topic", "/yoloe/plot")
        visualization_topic = rospy.get_param("~visualization_topic", "/yoloe/plot_image")
        self.bounding_boxes_pub = rospy.Publisher(bounding_boxes_topic, BoundingBoxes, queue_size=2)
        self.visualization_pub = rospy.Publisher(visualization_topic, Image, queue_size=1)

        self.tracking_status = {}
        self.status_sub = rospy.Subscriber(
            rospy.get_param("~tracking_status_topic", "/tracking/status"), String,
            self._status_callback, queue_size=1)
        self.frame_queue = Queue(maxsize=1)
        self.running = True
        self.worker = threading.Thread(target=self._processing_loop, daemon=True)
        self.worker.start()

        self._create_subscribers()
        rospy.on_shutdown(self.stop)
        rospy.loginfo(
            "Tracking YOLOE bbox publisher is ready on %s (%s).",
            bounding_boxes_topic,
            self.device,
        )

    def _status_callback(self, message):
        try:
            self.tracking_status = json.loads(message.data)
        except (ValueError, TypeError):
            pass

    def _resolve_yoloe_root(self):
        package_path = Path(rospkg.RosPack().get_path("tracking_detector")).resolve()
        default_root = package_path / "vendor" / "yoloe"
        root = Path(
            rospy.get_param("~yoloe_root", os.environ.get("YOLOE_ROOT", str(default_root)))
        ).expanduser().resolve()
        if not (root / "ultralytics").is_dir():
            raise RuntimeError("YOLOE source directory was not found: {}".format(root))

        for source_dir in (
            root,
            root / "third_party" / "ml-mobileclip",
            root / "third_party" / "CLIP",
        ):
            sys.path.insert(0, str(source_dir))
        return root

    def _load_model(self):
        try:
            import torch
            from ultralytics import YOLOE
        except ImportError as exc:
            raise RuntimeError(
                "YOLOE Python dependencies are missing. See tracking_detector/README.md for Python dependencies."
            ) from exc

        requested_device = str(rospy.get_param("~device", "auto"))
        if requested_device == "auto":
            self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        else:
            self.device = requested_device
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested, but PyTorch cannot access a CUDA device")

        model_path = Path(
            rospy.get_param(
                "~model_path", str(self.yoloe_root / "prompt/yoloe_pretrain/yoloe-11m-seg.pt")
            )
        ).expanduser().resolve()
        prompt_path = Path(
            rospy.get_param("~prompt_path", str(self.yoloe_root / "prompt/prompt.txt"))
        ).expanduser().resolve()
        if not model_path.is_file():
            raise RuntimeError("YOLOE model was not found: {}".format(model_path))
        if not prompt_path.is_file():
            raise RuntimeError("YOLOE prompt file was not found: {}".format(prompt_path))

        with prompt_path.open("r", encoding="utf-8") as prompt_file:
            self.class_names = [line.strip() for line in prompt_file if line.strip()]
        if not self.class_names:
            raise RuntimeError("YOLOE prompt file is empty: {}".format(prompt_path))

        rospy.loginfo("Loading YOLOE model %s with %d prompts.", model_path, len(self.class_names))
        previous_directory = os.getcwd()
        try:
            # MobileCLIP resolves its local weight name from the working directory.
            os.chdir(str(self.yoloe_root))
            self.model = YOLOE(str(model_path))
            self.model.to(self.device)
            self.model.model.eval()
            with torch.no_grad():
                text_embeddings = self.model.get_text_pe(self.class_names)
                self.model.set_classes(self.class_names, text_embeddings)
        finally:
            os.chdir(previous_directory)

        self.torch = torch

        warmup_runs = int(rospy.get_param("~warmup_runs", 1))
        warmup_image = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)
        for _ in range(max(0, warmup_runs)):
            self.model.predict(
                warmup_image,
                conf=self.confidence,
                imgsz=(self.output_height, self.output_width),
                device=self.device,
                verbose=False,
            )

    def _create_subscribers(self):
        rgb_topic = rospy.get_param("~rgb_topic", "/camera0/color/image/compressed")
        rgb_type = CompressedImage if rospy.get_param("~rgb_compressed", True) else Image
        # Detection does not need odometry. The estimator matches historical poses.
        self.rgb_sub = rospy.Subscriber(rgb_topic, rgb_type,
            lambda msg: self._rgb_odom_callback(msg, None), queue_size=1,
            buff_size=8*1024*1024)
        rospy.loginfo("Subscribing to RGB=%s.", rgb_topic)

    def _rgb_odom_callback(self, rgb_msg, odom_msg):
        frame = (rgb_msg, odom_msg)
        try:
            self.frame_queue.put_nowait(frame)
        except Full:
            try:
                self.frame_queue.get_nowait()
            except Empty:
                pass
            self.frame_queue.put_nowait(frame)

    def _decode_rgb(self, message):
        if isinstance(message, CompressedImage):
            return self.bridge.compressed_imgmsg_to_cv2(message, "bgr8")
        return self.bridge.imgmsg_to_cv2(message, "bgr8")

    def _prepare_frame(self, rgb_msg):
        rgb = self._decode_rgb(rgb_msg)
        output_size = (self.output_width, self.output_height)
        if (rgb.shape[1], rgb.shape[0]) != output_size:
            rgb = cv2.resize(rgb, output_size, interpolation=cv2.INTER_LINEAR)

        compressed_rgb = self.bridge.cv2_to_compressed_imgmsg(rgb, dst_format="jpg")
        compressed_rgb.header = rgb_msg.header
        return rgb, compressed_rgb

    def _processing_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                frame = self.frame_queue.get(timeout=0.5)
            except Empty:
                continue

            try:
                self._process_frame(*frame)
            except (CvBridgeError, ValueError) as exc:
                rospy.logerr_throttle(1.0, "Could not prepare RGB frame: %s", exc)
            except Exception as exc:
                rospy.logerr_throttle(1.0, "YOLOE frame processing failed: %s", exc)

    def _process_frame(self, rgb_msg, odom_msg):
        rgb, compressed_rgb = self._prepare_frame(rgb_msg)
        with self.torch.no_grad():
            results = self.model.predict(
                rgb,
                conf=self.confidence,
                imgsz=(self.output_height, self.output_width),
                device=self.device,
                retina_masks=False,
                verbose=self.debug,
            )
        if not results:
            empty = BoundingBoxes()
            empty.header = rgb_msg.header
            empty.image_header = rgb_msg.header
            self.bounding_boxes_pub.publish(empty)
            return

        result = results[0]
        if self.publish_visualization:
            visualization = result.plot(boxes=True, masks=False, conf=True, labels=True)
            status = self.tracking_status
            age = status.get("observation_age")
            age_text = "{:.2f}s".format(age) if isinstance(age, (int,float)) else "n/a"
            label = "{} | age {} | {}".format(status.get("state","acquiring"),
                                                age_text, status.get("range_method","none"))
            color = (0,220,0) if status.get("valid") else (0,160,255)
            cv2.putText(visualization, label, (8,22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 1, cv2.LINE_AA)
            visualization_msg = self.bridge.cv2_to_imgmsg(visualization, "bgr8")
            visualization_msg.header = rgb_msg.header
            self.visualization_pub.publish(visualization_msg)

        if result.boxes is None:
            empty = BoundingBoxes()
            empty.header = rgb_msg.header
            empty.image_header = rgb_msg.header
            self.bounding_boxes_pub.publish(empty)
            return

        class_ids = result.boxes.cls.detach().cpu().numpy().astype(int)
        confidences = result.boxes.conf.detach().cpu().numpy()
        boxes = result.boxes.xyxy.detach().cpu().numpy()
        detection_count = min(len(class_ids), len(confidences), len(boxes))

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = rgb_msg.header
        bounding_boxes.image_header = rgb_msg.header

        for index in range(detection_count):
            class_id = class_ids[index]
            if class_id < 0 or class_id >= len(self.class_names):
                rospy.logwarn_throttle(1.0, "YOLOE returned invalid class id %d.", class_id)
                continue
            x_min, y_min, x_max, y_max = boxes[index]
            detection_box = BoundingBox()
            detection_box.probability = float(confidences[index])
            detection_box.xmin = int(np.clip(np.floor(x_min), 0, self.output_width - 1))
            detection_box.ymin = int(np.clip(np.floor(y_min), 0, self.output_height - 1))
            detection_box.xmax = int(np.clip(np.ceil(x_max), 0, self.output_width))
            detection_box.ymax = int(np.clip(np.ceil(y_max), 0, self.output_height))
            detection_box.id = int(class_id)
            detection_box.Class = self.class_names[class_id]
            bounding_boxes.bounding_boxes.append(detection_box)

        self.bounding_boxes_pub.publish(bounding_boxes)

    def stop(self):
        if not self.running:
            return
        self.running = False
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)


def main():
    rospy.init_node("tracking_yoloe")
    node = None
    try:
        node = YoloeSemanticPublisher()
        rospy.spin()
    except (RuntimeError, OSError) as exc:
        rospy.logfatal("Failed to start YOLOE semantic publisher: %s", exc)
        raise
    finally:
        if node is not None:
            node.stop()


if __name__ == "__main__":
    main()
