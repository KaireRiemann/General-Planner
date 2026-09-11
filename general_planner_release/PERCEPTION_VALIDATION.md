# Gate / Tracking release validation — 2026-09-11

Build and all executable tests ran inside the existing `ros1_noetic` container.
Source changes for this delivery concern packaging, launch wiring and tests;
the perception algorithms and planner algorithms were not changed by this task.
The release binaries/configs also contain the workspace's existing changes.

## Passed

- Release build including `aperture_detector`, `tracking_detector`, MapManager,
  General Planner and RViz plugin; shared runtime messages refreshed.
- Relocated release: ROS package/node discovery, generated messages, ldd with no
  missing libraries or devel/build links, and runtime/Unity launch parsing.
- Gate synthetic point clouds: opening geometry, timestamps, invalidation on
  stale input/disable, solid-wall rejection and area-quality gating.
- Gate VLM client: prompts, compressed image, local mocked HTTP service,
  pixel polygon and timestamp preservation. No real VLM service was called.
- Real CPU YOLOE -> bbox -> Python state estimator -> C++ prediction:
  8 car detections, 1 empty detection, 6 target odometry messages, 5 predictions
  in the relocated-directory test. Counts are fixture observations, not accuracy metrics.
- Gate -> Tracking -> state2state -> Gate lifecycle; stale status shuts down
  detection. Actual child executable/script paths stayed inside the relocated
  release even when parent ROS/CMake paths pointed at the source workspace.
- Additional existing target-route, tracking-brake and coverage-guidance self-tests.

`tests/test_release_perception.py --runtime` reproduces the release-specific
checks using its own master on 127.0.0.1:11329. It refuses an occupied listener.
No Planner, trajectory server or Unity command bridge is started by these tests.
Existing masters on 11311/11381 were not stopped or used for synthetic traffic.

The first test iteration encountered a stale master registration from its prior
geometry fixture. The test now checks live node XMLRPC endpoints rather than
registry names and permits TIME_WAIT reuse without accepting an occupied port.
No runtime behavior was weakened to make tests pass.

## Deployment boundary

Both perception packages, Tracking weights, Unity bridge and ROS-TCP endpoint
are included. ROS/system libraries and Python/PyTorch dependencies remain
environment requirements; see PERCEPTION_ENVIRONMENT.md.
Gate's VLM model service remains external. Use `perceptor_vlm:=false` for pure
point-cloud detection. The Unity gate overlay is scene-specific; it is not a
real-robot safety configuration. Real VLM, GPU inference, Unity dynamics and
physical flight were not validated here.
