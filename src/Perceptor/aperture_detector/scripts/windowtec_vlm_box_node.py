#!/usr/bin/env python3
"""ROS node for VLM window quadrilateral detection."""

from __future__ import annotations

import base64
import math
import os
import sys
import threading
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32, PolygonStamped
from sensor_msgs.msg import CompressedImage, Image


import rospkg

WINDOWTEC_ROOT = Path(rospkg.RosPack().get_path("aperture_detector")) / "scripts" / "windowtec_vlm_window_corners"
if str(WINDOWTEC_ROOT) not in sys.path:
    sys.path.insert(0, str(WINDOWTEC_ROOT))

from run import build_messages, parse_windows, response_format  # noqa: E402
from windowtec_core.io_utils import read_text  # noqa: E402
from windowtec_core.model_client import ModelClientError, OpenAICompatibleClient  # noqa: E402


def polygon_area(points: list[list[float]]) -> float:
    return 0.5 * abs(
        sum(
            points[i][0] * points[(i + 1) % len(points)][1]
            - points[(i + 1) % len(points)][0] * points[i][1]
            for i in range(len(points))
        )
    )


def signed_area(points: list[list[float]]) -> float:
    return 0.5 * sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points))
    )


def cross(a: list[float], b: list[float], c: list[float]) -> float:
    return (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])


def distance(a: list[float], b: list[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def is_projected_rectangle_like(
    points: list[list[float]],
    image_width: int,
    image_height: int,
    min_area_ratio: float,
    min_edge_px: float,
    min_corner_angle_deg: float,
    max_corner_angle_deg: float,
    max_edge_ratio: float,
) -> bool:
    if len(points) != 4:
        return False
    if len({(round(p[0], 3), round(p[1], 3)) for p in points}) != 4:
        return False
    if any(p[0] < 0 or p[0] > image_width or p[1] < 0 or p[1] > image_height for p in points):
        return False

    area = polygon_area(points)
    if area < max(1.0, min_area_ratio * image_width * image_height):
        return False

    signs = [cross(points[i - 1], points[i], points[(i + 1) % 4]) for i in range(4)]
    if not (all(v > 1e-6 for v in signs) or all(v < -1e-6 for v in signs)):
        return False

    edges = [distance(points[i], points[(i + 1) % 4]) for i in range(4)]
    if min(edges) < min_edge_px:
        return False
    if max(edges) / max(1e-6, min(edges)) > max_edge_ratio:
        return False

    min_cos = math.cos(math.radians(max_corner_angle_deg))
    max_cos = math.cos(math.radians(min_corner_angle_deg))
    for i in range(4):
        prev_pt = points[i - 1]
        cur_pt = points[i]
        next_pt = points[(i + 1) % 4]
        v1 = [prev_pt[0] - cur_pt[0], prev_pt[1] - cur_pt[1]]
        v2 = [next_pt[0] - cur_pt[0], next_pt[1] - cur_pt[1]]
        n1 = math.hypot(v1[0], v1[1])
        n2 = math.hypot(v2[0], v2[1])
        if n1 < 1e-6 or n2 < 1e-6:
            return False
        cos_angle = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
        if cos_angle < min_cos or cos_angle > max_cos:
            return False

    return abs(signed_area(points)) > 1e-6


class WindowTecVlmBoxNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_msg: Image | CompressedImage | None = None
        self.processing = False

        self.image_topic = rospy.get_param("~image_topic", "/camera0/color/image_raw")
        self.image_is_compressed = bool(
            rospy.get_param("~image_is_compressed", self.image_topic.endswith("/compressed"))
        )
        self.box_topic = rospy.get_param("~box_topic", "~window_box")
        self.overlay_topic = rospy.get_param("~overlay_topic", "~overlay_image")
        self.detection_period = float(rospy.get_param("~detection_period", 3.0))
        self.coordinate_scale = int(rospy.get_param("~coordinate_scale", 1000))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self.use_response_format = bool(rospy.get_param("~use_response_format", True))

        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.002))
        self.min_edge_px = float(rospy.get_param("~min_edge_px", 8.0))
        self.min_corner_angle_deg = float(rospy.get_param("~min_corner_angle_deg", 25.0))
        self.max_corner_angle_deg = float(rospy.get_param("~max_corner_angle_deg", 155.0))
        self.max_edge_ratio = float(rospy.get_param("~max_edge_ratio", 12.0))

        base_url = rospy.get_param("~base_url", "http://127.0.0.1:18000/v1")
        model = rospy.get_param("~model", "auto")
        api_key = rospy.get_param("~api_key", os.environ.get("WINDOWTEC_API_KEY", "EMPTY"))
        timeout = float(rospy.get_param("~timeout", 120.0))
        max_retries = int(rospy.get_param("~max_retries", 1))
        max_tokens = int(rospy.get_param("~max_tokens", 1024))

        system_prompt_path = Path(rospy.get_param("~system_prompt", str(WINDOWTEC_ROOT / "system_prompt.md")))
        task_prompt_path = Path(rospy.get_param("~task_prompt", str(WINDOWTEC_ROOT / "task_prompt.md")))
        self.system_prompt = read_text(system_prompt_path)
        self.task_prompt = read_text(task_prompt_path)

        self.client = None
        self.client_options = dict(
            base_url=base_url,
            api_key=api_key,
            model=model,
            timeout=timeout,
            temperature=0.0,
            top_p=1.0,
            max_tokens=max_tokens,
            max_retries=max_retries,
            disable_thinking=True,
        )

        self.pub = rospy.Publisher(self.box_topic, PolygonStamped, queue_size=1)
        self.overlay_pub = rospy.Publisher(self.overlay_topic, Image, queue_size=1)
        image_msg_type = CompressedImage if self.image_is_compressed else Image
        self.sub = rospy.Subscriber(self.image_topic, image_msg_type, self.image_callback, queue_size=1, buff_size=2**24)
        self.timer = rospy.Timer(rospy.Duration(max(0.1, self.detection_period)), self.timer_callback)

        rospy.loginfo(
            "[windowtec_vlm_box_node] image_topic=%s compressed=%s box_topic=%s period=%.2fs model=%s",
            self.image_topic,
            self.image_is_compressed,
            self.box_topic,
            self.detection_period,
            model,
        )

    def image_callback(self, msg: Image | CompressedImage) -> None:
        with self.lock:
            self.latest_msg = msg

    def timer_callback(self, _event: Any) -> None:
        with self.lock:
            if self.processing or self.latest_msg is None:
                return
            msg = self.latest_msg
            self.processing = True

        thread = threading.Thread(target=self.process_image, args=(msg,), daemon=True)
        thread.start()

    def decode_image(self, msg: Image | CompressedImage) -> Any:
        if isinstance(msg, CompressedImage):
            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            array = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if array is None:
                raise ValueError("failed to decode compressed image")
            return array
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_image(self, msg: Image | CompressedImage) -> None:
        try:
            if self.client is None:
                self.client = OpenAICompatibleClient(**self.client_options)
            cv_image = self.decode_image(msg)
            image_height, image_width = cv_image.shape[:2]
            ok, encoded = cv2.imencode(
                ".jpg",
                cv_image,
                [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, self.jpeg_quality))],
            )
            if not ok:
                rospy.logwarn("[windowtec_vlm_box_node] failed to JPEG-encode image")
                return

            image_url = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
            messages = build_messages(
                self.system_prompt,
                self.task_prompt,
                str(msg.header.stamp.to_nsec()),
                image_width,
                image_height,
                image_url,
                self.coordinate_scale,
            )

            fmt = response_format(self.coordinate_scale) if self.use_response_format else None
            try:
                result = self.client.chat(messages, response_format=fmt)
            except ModelClientError as exc:
                if not self.use_response_format:
                    raise
                rospy.logwarn("[windowtec_vlm_box_node] schema request failed, retrying without response_format: %s", exc)
                result = self.client.chat(messages, response_format=None)

            _payload, windows, errors, parse_error, _format_valid = parse_windows(
                result.content,
                image_width,
                image_height,
                self.coordinate_scale,
            )
            if parse_error:
                rospy.logwarn("[windowtec_vlm_box_node] invalid model output: %s", parse_error)
                return
            if errors:
                rospy.logwarn_throttle(10.0, "[windowtec_vlm_box_node] ignored malformed windows: %s", errors[:2])

            candidates = []
            for window in windows:
                pixel_points = [[float(x), float(y)] for x, y in window["pixel_points"]]
                if not is_projected_rectangle_like(
                    pixel_points,
                    image_width,
                    image_height,
                    self.min_area_ratio,
                    self.min_edge_px,
                    self.min_corner_angle_deg,
                    self.max_corner_angle_deg,
                    self.max_edge_ratio,
                ):
                    continue
                candidates.append((polygon_area(pixel_points), pixel_points))

            if not candidates:
                rospy.loginfo("[windowtec_vlm_box_node] no valid projected-rectangle-like window")
                return

            _area, best_points = max(candidates, key=lambda item: item[0])
            self.publish_box(best_points, msg.header)
            self.publish_overlay(cv_image, best_points, msg.header)
            rospy.loginfo(
                "[windowtec_vlm_box_node] published largest window area=%.1f px^2 latency=%dms",
                _area,
                result.latency_ms,
            )
        except (CvBridgeError, ModelClientError, RuntimeError, ValueError) as exc:
            rospy.logwarn("[windowtec_vlm_box_node] detection failed: %s", exc)
        finally:
            with self.lock:
                self.processing = False

    def publish_box(self, pixel_points: list[list[float]], header: Any) -> None:
        msg = PolygonStamped()
        msg.header = header
        for x, y in pixel_points:
            point = Point32()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            msg.polygon.points.append(point)
        self.pub.publish(msg)

    def publish_overlay(self, cv_image: Any, pixel_points: list[list[float]], header: Any) -> None:
        overlay = cv_image.copy()
        int_points = [(int(round(x)), int(round(y))) for x, y in pixel_points]
        for i in range(4):
            cv2.line(overlay, int_points[i], int_points[(i + 1) % 4], (0, 180, 255), 2, cv2.LINE_AA)
        labels = ("TL", "TR", "BR", "BL")
        for label, point in zip(labels, int_points):
            cv2.circle(overlay, point, 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(
                overlay,
                label,
                (point[0] + 7, point[1] - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        image_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        image_msg.header = header
        self.overlay_pub.publish(image_msg)


def main() -> None:
    rospy.init_node("windowtec_vlm_box_node")
    WindowTecVlmBoxNode()
    rospy.spin()


if __name__ == "__main__":
    main()
