#!/usr/bin/env python3
"""Check local inference assets/imports without requiring a ROS master."""
import os
import sys
from pathlib import Path
import rospkg

def main():
    root = Path(rospkg.RosPack().get_path("tracking_detector")) / "vendor/yoloe"
    for part in (root, root / "third_party/ml-mobileclip", root / "third_party/CLIP"):
        sys.path.insert(0, str(part))
    for item in (root / "mobileclip_blt.pt", root / "prompt/yoloe_pretrain/yoloe-11m-seg.pt"):
        if not item.is_file():
            raise RuntimeError("Missing model asset: " + str(item))
    import torch
    import mobileclip
    import clip
    import ultralytics
    from ultralytics import YOLOE
    from tracking_detector.msg import BoundingBoxes
    import cv_bridge
    for module in (ultralytics, mobileclip, clip):
        path = Path(module.__file__).resolve()
        if root.resolve() not in path.parents:
            raise RuntimeError("External inference module unexpectedly imported: " + str(path))
        print(module.__name__, path)
    print("torch", torch.__version__, "CUDA available:", torch.cuda.is_available())
    print("tracking_detector assets and ROS/Python imports OK")

if __name__ == "__main__":
    main()
