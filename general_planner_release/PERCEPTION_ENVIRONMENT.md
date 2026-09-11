# Perception runtime environment

Packaging/validation container: `ros1_noetic`, ROS Noetic, Ubuntu 20.04 x86_64, Python 3.8.
The following distributions were present during the 2026-09-11 release validation:

| Distribution | Version |
| --- | --- |
| torch | 2.4.1+cpu |
| torchvision | 0.19.1+cpu |
| timm | 0.9.5 |
| numpy | 1.24.4 |
| opencv-python | 4.8.1.78 |
| scipy | 1.10.0 |
| Pillow | 10.4.0 |

This is an environment record, not a complete pip lockfile or an instruction to
replace a working CUDA environment. The release includes YOLOE, MobileCLIP and
CLIP implementation files and weights under `src/tracking_detector/vendor/yoloe`.
Run its `scripts/check_runtime.py` after sourcing this release to verify imports.
System ROS libraries, cv_bridge and Python/PyTorch dependencies must be installed.
The Gate VLM model/server is external; only its client and prompts are bundled.
CPU inference is covered by the bundled perception test; GPU deployment is not.
