# Shared Vision Core Agent Guide

## Project role

`vision_core` is the shared ROS-independent C++ perception, mission-control, and command-generation library.

It is used by multiple execution environments, including:

- the real ROS2 `vision` package
- simulator/Python callers through the C API

It is a library, not a standalone ROS node.

## Hard dependency boundary

Keep this library independent of runtime-specific robotics middleware and inference infrastructure.

Do not introduce dependencies on:

- ROS2 / rclcpp
- sensor_msgs
- geometry_msgs
- cv_bridge
- TensorRT
- CUDA
- camera drivers

Avoid adding OpenCV unless an explicit architectural decision requires it; the current core API is intentionally independent of the camera/inference stack.

Hardware and transport adaptation belongs in `vision`.

## Single source of algorithm truth

Shared algorithm configuration lives in:

`config/vision_algorithm.yaml`

Do not duplicate canonical numerical algorithm defaults in the ROS adapter or simulator.

Configuration structs transport values; the YAML is the canonical shared configuration source.

If required configuration is missing or invalid, preserve the existing fail-fast behavior rather than silently inventing fallback parameters.

## Mission architecture

`MissionController` is the central mission-state-machine entry point.

Callers should provide observations and execution feedback, then consume the returned `ControlCommand`.

External callers must not independently perform:

- mission-priority selection
- mission locking/unlocking
- controller reset sequencing
- pickup/goal/hurdle state transitions
- action completion routing

Individual controllers own their mission-specific behavior.

`MissionController` owns cross-mission coordination.

## Command lifecycle

Preserve the distinction between:

- continuous velocity commands
- locomotion/P2P actions
- mission/discrete actions
- stationary actions
- camera requests

ACK/DONE and action-ID behavior are part of the public control contract.

Do not change action lifecycle semantics as a side effect of unrelated controller work.

## P2P behavior

P2P quantization occurs after continuous mission/control logic has produced the selected velocity command.

Do not move mission logic into `P2pMotionQuantizer`.

Preserve access to the pre-quantization motion command for simulator/velocity-compatible consumers.

## C / C++ API compatibility

`c_api` is consumed by simulator/Python clients.

Treat exported C API structures and functions as compatibility-sensitive interfaces.

Before changing an existing C API:

1. inspect current users,
2. determine whether ABI/API compatibility can be preserved,
3. prefer additive/versioned interfaces where practical.

Do not casually rename or remove existing exported functions or fields.

## Determinism

Keep core behavior deterministic for identical:

- configuration
- observations
- controller state
- execution feedback

Do not introduce hidden ROS state, wall-clock dependencies, global runtime state, or hardware access into core decision logic.

## Tests

The tests directory is part of the product contract, not disposable generated code.

Current test areas include:

- config loading
- ball controller
- hurdle controller
- goal controller
- line detection stability
- control-command lifecycle
- P2P motion quantization
- mission coordination
- perception pipeline
- object association tracking

Behavior changes should update or add focused tests.

Do not delete or weaken a test merely because a new implementation fails it.

## Verification

Standard verification:

`cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`

`cmake --build build -j`

`ctest --test-dir build --output-on-failure`

For changes to shared behavior, run the full CTest suite before claiming completion.

When installation behavior changes, also verify:

`cmake --install build --prefix install`

## Repository ownership

Put code here when the behavior must be shared between real and simulated environments.

Put code in `vision` when it concerns:

- ROS topics
- camera input
- image transport
- CUDA preprocessing
- TensorRT
- visualization
- hardware/runtime adaptation

When uncertain, preserve the core's ROS-independent boundary.

## Scope discipline

For a targeted task:

1. inspect the relevant API and implementation,
2. inspect its focused tests,
3. understand public callers,
4. make the smallest coherent change,
5. run the affected tests,
6. run the full test suite when shared behavior changed.

Avoid unrelated refactors unless explicitly requested.
