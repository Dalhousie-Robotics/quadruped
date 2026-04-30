# ADR-002: Use MuJoCo for Robot Simulation

**Date:** 2026-04-29
**Status:** Accepted
**Deciders:** Software Team Lead

---

## Context

We need a simulation environment to develop and test gait control, kinematics, and locomotion algorithms before deploying to the physical robot. Requirements:
- Accurate contact/collision physics (critical for legged robots)
- Python API (team's primary language)
- Windows support (team's OS)
- Free / open-source

## Decision

Use **MuJoCo 3.x** with its Python bindings (`mujoco` PyPI package).

## Rationale

| Option | Contact physics | Python API | Windows | Notes |
|---|---|---|---|---|
| MuJoCo | Excellent | Yes | Yes | MIT Cheetah, ANYmal, Spot all developed with it |
| Gazebo (ROS) | Good | Yes (via ROS) | Poor | Requires ROS; Windows support is painful |
| PyBullet | Decent | Yes | Yes | Less maintained, less accurate contacts |
| Isaac Sim | Good | Yes | Yes | Requires NVIDIA GPU, complex setup |
| Webots | Decent | Yes | Yes | Less used in legged robotics research |

MuJoCo is the de facto standard for legged robot simulation in research. Its contact model is best-in-class for foot-ground interactions. It is free since DeepMind open-sourced it.

## Consequences

- Robot model is defined in **MJCF** (MuJoCo XML), not URDF. URDF can be converted.
- Simulation code lives in `src/simulation/` and uses the `mujoco` Python package.
- The `SimMotorInterface` class wraps MuJoCo actuators to match the real `SerialMotorInterface` API.
- All gait and kinematics algorithms must work against the `MotorInterface` abstraction, making them simulator-agnostic.
