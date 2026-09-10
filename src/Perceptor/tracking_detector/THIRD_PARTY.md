# Source provenance

This package consolidates local tracking components at their 2026-09-10 state.

* `scripts/yoloe_bbox_node.py`: adapted from `scene_graph_copy/ws_main/src/planner/scene_graph/scripts/yoloe_semantic_publisher.py`. Removed scene graph messages, masks, semantic embeddings and motion classification; retained model loading, synchronized inputs, latest-frame worker, inference and bbox conversion. The source scene_graph manifest declares its license as `TODO`; this notice does not assign a new license to that source.
* `src/target_ekf_node.cpp` and `src/target_path_predictor_node.cpp`: copied from `elastic-tracker-with-label/src/detection/target_ekf/src`, with bbox namespace adapted to this package. Source manifest declares GPLv3; see `LICENSE.target_ekf`.
* `msg/BoundingBox.msg` and `msg/BoundingBoxes.msg`: copied from that workspace's object_detection_msgs. Message package namespace is now tracking_detector.
* `config/camera.yaml`: copied from target_ekf; `config/prompts.txt`: copied from scene_graph_copy/yoloe/prompt/prompt.txt.
* `vendor/yoloe`: local inference implementation and its bundled third-party sources, copied rather than symlinked. Original license files are retained, including Ultralytics AGPL-3.0 and MobileCLIP/CLIP licenses. Python caches and repository metadata excluded.
* Model files: copied from the existing local YOLOE installation. Their hashes/sizes are in `config/model_assets.json`; no model download or replacement occurred.
* `tests/data/car.jpg`: extracted from local `scene_graph_copy/tracker_jitter.bag`, /yoloe/encodemask/current_rgb, bag time 1787043677.138133349. Used only as an offline integration fixture.

The package manifest lists the known GPL/AGPL components, not a relicensing of third-party code or weights.
