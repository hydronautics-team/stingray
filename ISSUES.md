# Issues

Review scope:

- First-party packages under `src/`, excluding `src/stingray_core`.
- No deep review of third-party code in
  `src/stingray_object_detection/ultralytics` or vendored
  `src/stingray_object_detection/yolov5`.

Validation limits:

- Static inspection only.
- Python syntax sanity check completed for first-party `.py` files.
- No local `/opt/ros` installation and no `colcon`, so build/runtime checks were
  not possible in this environment.

## Repository / Cross-package

- [high] Build and packaging strategy is inconsistent across reviewed packages: mixed `ament_python` and `ament_cmake`, missing runtime dependencies, placeholder metadata, and conflicting install conventions make reproducible deployment unclear. Refs: `src/stingray_utils/CMakeLists.txt:7-56`, `src/stingray_object_detection/setup.py:12-29`, `src/stingray_missions/package.xml:10-20`, `src/stingray_recorder/package.xml:10-20`.
- [medium] Test coverage is limited to generated lint scaffolding; there are no functional or integration tests for launch, FSM behavior, detector output, recorder control, or action-server success criteria. Refs: `src/stingray_missions/test/test_flake8.py:1-25`, `src/stingray_object_detection/test/test_pep257.py:1-23`, `src/stingray_recorder/test/test_flake8.py:1-25`.

## stingray_cam

- [low] The package is effectively a calibration/resource package but still carries generated placeholder metadata instead of a documented ownership/usage contract. Refs: `src/stingray_cam/package.xml:6-8`, `src/stingray_cam/setup.py:20-21`.

## stingray_launch

- [critical] `control.launch.py` starts `hydroacoustic_centering_twist_action_server`, but `stingray_movement` does not build or install that executable; full-stack bring-up will fail during launch. Refs: `src/stingray_launch/launch/control.launch.py:179-191`, `src/stingray_movement/CMakeLists.txt:32-53`.
- [medium] `missions.launch.py` duplicates `uv_state_topic`, keeps unused imports, and overlaps semantically with `control.launch.py`, which suggests configuration drift between the two bring-up paths. Refs: `src/stingray_launch/launch/missions.launch.py:1-14`, `src/stingray_launch/launch/missions.launch.py:49-64`, `src/stingray_launch/launch/control.launch.py:7-207`.
- [medium] Launch files expose parameters for interfaces that are not consistently implemented downstream, so operators can enable features that the reviewed codebase cannot actually satisfy. Refs: `src/stingray_launch/launch/control.launch.py:39-41`, `src/stingray_launch/launch/control.launch.py:110-111`, `src/stingray_launch/launch/missions.launch.py:46-48`.

## stingray_missions

- [critical] FSM timeouts are wired through `create_timer(..., self._state_expired)` where `_state_expired` is an `async def`; `rclpy` timers expect a normal callback, so timeout transitions may never execute as intended. Refs: `src/stingray_missions/stingray_missions/fsm.py:188-193`, `src/stingray_missions/stingray_missions/fsm.py:245-249`.
- [high] The default `mission_package_names` parameter is malformed as a one-element string array containing `'sauvc_missions, stingray_missions'`, which breaks default scenario-package discovery unless launch-time overrides are supplied. Refs: `src/stingray_missions/stingray_missions/fsm_node.py:33-35`.
- [high] YAML caches are keyed only by config filename, not by package name, so same-named mission/scenario configs from different packages can collide in memory. Refs: `src/stingray_missions/stingray_missions/descriptions.py:134-151`, `src/stingray_missions/stingray_missions/descriptions.py:266-284`.
- [high] `ScenarioDescription` mutates the cached `transitions` list in-place by appending a FAILED transition, which can accumulate duplicate transitions across repeated loads. Refs: `src/stingray_missions/stingray_missions/descriptions.py:203-238`.
- [high] The action layer imports `HydroacousticCenteringTwistAction`, but there is no matching state-action implementation or registration, even though launch files expose the parameter. Refs: `src/stingray_missions/stingray_missions/action.py:10`, `src/stingray_missions/stingray_missions/action.py:449-462`.
- [high] `package.xml` does not declare several Python/runtime dependencies used directly by the package (`rclpy`, `std_srvs`, `ament_index_python`, `transitions`). Refs: `src/stingray_missions/package.xml:10-20`, `src/stingray_missions/stingray_missions/action.py:1-14`, `src/stingray_missions/stingray_missions/fsm.py:1-12`.
- [medium] `qr_trigger_node` only checks service availability once at startup and then fires async requests without result/error handling, which makes QR-trigger behavior brittle during service restarts. Refs: `src/stingray_missions/stingray_missions/qr_trigger_node.py:17-25`, `src/stingray_missions/stingray_missions/qr_trigger_node.py:51-54`.
- [medium] FSM execution is driven by tight polling loops with `spin_once(..., timeout_sec=0)` and `sleep(1e-4)`, which is CPU-heavy and makes timing/debugging harder than an executor-driven design. Refs: `src/stingray_missions/stingray_missions/fsm_node.py:10-30`, `src/stingray_missions/stingray_missions/fsm_node.py:68-74`.

## stingray_movement

- [critical] Depth, roll, pitch, and yaw completion checks are stubbed to `true`, so movement actions can report success without ever verifying that the vehicle reached its target state. Refs: `src/stingray_movement/include/stingray_movement/AbstractTwistActionServer.h:19-101`.
- [high] Movement servers use `current_uv_state` immediately without guarding against the case where no UV-state message has been received yet, so first action execution can run against default-initialized state. Refs: `src/stingray_movement/include/stingray_movement/AbstractTwistActionServer.h:158-187`, `src/stingray_movement/src/TwistActionServer.cpp:48-49`, `src/stingray_movement/src/BboxSearchTwistActionServer.cpp:139-141`.
- [medium] `BboxSearchTwistActionServer` has timeout/duration enforcement commented out, so search failure currently depends only on yaw limit or shutdown rather than mission-level time bounds. Refs: `src/stingray_movement/src/BboxSearchTwistActionServer.cpp:131-173`.
- [medium] Action threads block on synchronous `.wait()` calls to services, which weakens cancellation semantics and makes shutdown behavior harder to reason about. Refs: `src/stingray_movement/src/TwistActionServer.cpp:53`, `src/stingray_movement/src/BboxCenteringTwistActionServer.cpp:278`, `src/stingray_movement/src/BboxSearchTwistActionServer.cpp:141`, `src/stingray_movement/src/BboxSearchTwistActionServer.cpp:186`, `src/stingray_movement/src/BboxSearchTwistActionServer.cpp:221`.

## stingray_devices

- [high] Service-unavailable logging references undeclared parameter `set_enable_device` instead of `set_device_srv`, so operational diagnostics are misleading exactly when the action server is already failing. Refs: `src/stingray_devices/src/DeviceActionServer.cpp:41-47`.
- [high] The action server reports success immediately after sending `SetDevice`; state verification and timeout/cancel handling are present only as commented code. Refs: `src/stingray_devices/src/DeviceActionServer.cpp:55-98`.

## stingray_interfaces

- [high] `HydroacousticCenteringTwistAction` is published as a first-class public interface, but the reviewed packages do not implement the corresponding action server or mission action. Refs: `src/stingray_interfaces/action/HydroacousticCenteringTwistAction.action:1-19`, `src/stingray_launch/launch/control.launch.py:179-191`, `src/stingray_missions/stingray_missions/action.py:10`.
- [medium] `DeviceAction.action` still contains placeholder comments instead of a stable device ID contract or named constants, which leaves downstream packages to coordinate magic numbers informally. Refs: `src/stingray_interfaces/action/DeviceAction.action:1-8`.

## stingray_object_detection

- [critical] `weights/bbox_attrs.yaml` is empty, but runtime code assumes a loaded object-attribute map; the first supported detection path can fail inside distance estimation. Refs: `src/stingray_object_detection/weights/bbox_attrs.yaml`, `src/stingray_object_detection/stingray_object_detection/yolo_detector_base.py:65-72`, `src/stingray_object_detection/stingray_object_detection/distance.py:21-46`, `src/stingray_object_detection/stingray_object_detection/yolov8_detector.py:102-114`.
- [critical] `setup.py` exports `yolov5_detector`, but the module does not exist in the first-party package tree. Refs: `src/stingray_object_detection/setup.py:25-29`.
- [high] Package installation only includes `weights/*.pt`, while runtime depends on `weights/yolov8.yaml` and `weights/bbox_attrs.yaml`; installed deployments will miss required config files. Refs: `src/stingray_object_detection/setup.py:12-17`, `src/stingray_object_detection/stingray_object_detection/yolo_detector_base.py:65-68`, `src/stingray_object_detection/stingray_object_detection/yolov8_detector.py:25-29`.
- [high] Detector callbacks can reach `detect()` before any `CameraInfo` has been cached for the topic, which can lead to `KeyError` or invalid distance math. Refs: `src/stingray_object_detection/stingray_object_detection/yolo_detector_base.py:103-105`, `src/stingray_object_detection/stingray_object_detection/yolo_detector_base.py:182-203`, `src/stingray_object_detection/stingray_object_detection/yolov8_detector.py:102-104`.
- [medium] Output publishing keeps only the highest-confidence bbox per class label, so multi-instance scenes are collapsed before downstream consumers ever see them. Refs: `src/stingray_object_detection/stingray_object_detection/yolov8_detector.py:83-145`.
- [medium] `package.xml` does not declare several Python runtime dependencies imported directly by the package (`torch`, `ultralytics`, `filterpy`, `ament_index_python`). Refs: `src/stingray_object_detection/package.xml:11-24`, `src/stingray_object_detection/stingray_object_detection/yolov8_detector.py:1-15`, `src/stingray_object_detection/stingray_object_detection/tracker.py:1-5`.

## stingray_recorder

- [high] The node imports `stingray_interfaces.msg.EnableTopic`, but `package.xml` does not declare a dependency on `stingray_interfaces` or `rclpy`; packaged installs can fail even before recorder logic starts. Refs: `src/stingray_recorder/package.xml:10-20`, `src/stingray_recorder/stingray_recorder/video_recorder_node.py:7-12`.
- [medium] `VideoWriter` is created with configured output dimensions, but frames are written without resize, so any source topic that does not already match those dimensions can produce invalid or dropped video. Refs: `src/stingray_recorder/stingray_recorder/video_recorder_node.py:100-109`.

## stingray_utils

- [critical] `stingray_utils` mixes `ament_cmake`, `ament_python_install_package`, a placeholder `setup.cfg`, and a separate `setup.py`; the package has no coherent install contract for its Python helpers. Refs: `src/stingray_utils/package.xml:14-28`, `src/stingray_utils/CMakeLists.txt:7-56`, `src/stingray_utils/setup.cfg:1-4`, `src/stingray_utils/setup.py:7-26`.
- [high] `AsyncSubscription` passes an `async def` callback directly to `create_subscription`, which `rclpy` does not execute as a coroutine callback. Refs: `src/stingray_utils/stingray_utils/acyncio.py:9-18`.
- [high] Utility modules still depend on ROS 1-era `rospkg` and a non-existent `stingray_resources` package, which makes the helper layer inconsistent with the ROS 2 package layout used elsewhere. Refs: `src/stingray_utils/stingray_utils/config.py:3`, `src/stingray_utils/stingray_utils/resources.py:3-9`.
- [high] `StingrayConfig` hardcodes `configs/ros.yaml`, and `setup.py` expects `configs/*.yaml`, but the reviewed `stingray_utils` tree does not contain a `configs/` directory. Refs: `src/stingray_utils/stingray_utils/config.py:40-42`, `src/stingray_utils/setup.py:11-16`.
- [high] `stingray_utils.launch.py` launches executable `stingray_recorder` from package `stingray_utils`, but no such entry point or binary exists in the package. Refs: `src/stingray_utils/launch/stingray_utils.launch.py:49-63`, `src/stingray_utils/CMakeLists.txt:20-56`, `src/stingray_utils/setup.py:24-25`.
- [medium] Legacy `video_recorder.py` duplicates recorder functionality that now lives in `stingray_recorder`, increasing maintenance drift and the chance of outdated launch wiring. Refs: `src/stingray_utils/stingray_utils/video_recorder.py:29-189`.
