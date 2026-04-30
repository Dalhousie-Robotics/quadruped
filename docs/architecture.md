# System Architecture

This document describes the software and firmware architecture of the Dal Robotics quadruped. It is the authoritative reference for how the system is structured and why.

---

## Overview

The system is divided into two compute domains:

| Domain | Hardware | Language | Responsibility |
|---|---|---|---|
| On-robot (embedded) | ESP32 | C++ | Real-time motor control, sensor reading |
| Off-robot (PC) | Windows PC | Python | Gait planning, simulation, high-level commands |

These two domains communicate over **USB Serial** during development and **WiFi UDP** when deployed.

---

## Layer Diagram

```
┌──────────────────────────────────────────────────────┐
│  LAYER 5: Autonomy & AI (ML Team)                    │
│  Person detection, campus navigation, conversation   │
│  Interface: see docs/ml_interface_agreement.md       │
├──────────────────────────────────────────────────────┤
│  LAYER 4: High-Level Control (PC / Python)           │
│  Gait scheduler, foot trajectory planner,            │
│  body pose controller, command dispatcher            │
├──────────────────────────────────────────────────────┤
│  LAYER 3: Kinematics (PC / Python)                   │
│  Forward kinematics (FK), Inverse kinematics (IK)    │
│  for all 4 legs                                      │
├──────────────────────────────────────────────────────┤
│  LAYER 2: Serial / WiFi Protocol                     │
│  JSON or binary packet: joint targets → ESP32        │
│  Joint states, IMU data ← ESP32                      │
├──────────────────────────────────────────────────────┤
│  LAYER 1: ESP32 Firmware (C++)                       │
│  Receives joint targets, runs PD control loop,       │
│  sends CAN frames, reads IMU, watchdog               │
├──────────────────────────────────────────────────────┤
│  LAYER 0: CAN Bus + Motors                           │
│  AK40-10 × 12, MIT Mini Cheetah CAN protocol        │
└──────────────────────────────────────────────────────┘
```

---

## Communication Protocol (PC ↔ ESP32)

### Command packet (PC → ESP32)
Sent at the gait control loop rate (~100 Hz from PC side).

```json
{
  "seq": 1042,
  "joints": {
    "FL": { "hip_ab": 0.0, "hip_fe": -0.5, "knee": 1.2 },
    "FR": { "hip_ab": 0.0, "hip_fe": -0.5, "knee": 1.2 },
    "RL": { "hip_ab": 0.0, "hip_fe": -0.5, "knee": 1.2 },
    "RR": { "hip_ab": 0.0, "hip_fe": -0.5, "knee": 1.2 }
  },
  "kp": 5.0,
  "kd": 0.5
}
```

All angles are in **radians**. `FL/FR/RL/RR` = Front-Left, Front-Right, Rear-Left, Rear-Right.

### State packet (ESP32 → PC)
Sent at the CAN feedback rate (~500 Hz, decimated to ~100 Hz over serial).

```json
{
  "seq": 1042,
  "joints": {
    "FL": { "hip_ab": { "pos": 0.0, "vel": 0.0, "torque": 0.1 }, ... },
    ...
  },
  "imu": { "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "ax": 0.0, "ay": 0.0, "az": -9.81 },
  "timestamp_ms": 54321
}
```

> Note: the exact wire format (JSON vs binary) is TBD and will be decided in ADR-003. JSON is used during development for debuggability; binary (e.g., MessagePack or custom struct) may be adopted for deployed performance.

---

## CAN Bus Architecture

Each AK40-10 motor has a unique CAN ID (1–12). The ESP32 uses its built-in **TWAI** peripheral (CAN 2.0B compatible).

Motor ID assignment:

| ID | Joint | Leg |
|---|---|---|
| 1 | Hip Abduction/Adduction | Front Left |
| 2 | Hip Flexion/Extension | Front Left |
| 3 | Knee | Front Left |
| 4 | Hip Abduction/Adduction | Front Right |
| 5 | Hip Flexion/Extension | Front Right |
| 6 | Knee | Front Right |
| 7 | Hip Abduction/Adduction | Rear Left |
| 8 | Hip Flexion/Extension | Rear Left |
| 9 | Knee | Rear Left |
| 10 | Hip Abduction/Adduction | Rear Right |
| 11 | Hip Flexion/Extension | Rear Right |
| 12 | Knee | Rear Right |

CAN bus speed: **1 Mbit/s** (AK40-10 default).

---

## MIT Mini Cheetah CAN Protocol

The AK40-10 uses the MIT Mini Cheetah actuator protocol. Each CAN frame is 8 bytes.

### Command frame (host → motor)
```
Byte [0:1]  position target    (uint16, range maps to -12.5 to +12.5 rad)
Byte [2:3]  velocity target    (uint16, range maps to -65.0 to +65.0 rad/s)
Byte [4]    kp                 (uint8,  range maps to 0 to 500 Nm/rad)
Byte [5]    kd                 (uint8,  range maps to 0 to 5 Nm·s/rad)
Byte [6:7]  feedforward torque (uint16, range maps to -18.0 to +18.0 Nm)
```

### Response frame (motor → host)
```
Byte [0]    Motor ID
Byte [1:2]  position           (uint16, same mapping)
Byte [3:4]  velocity           (uint16, same mapping)
Byte [5:6]  torque             (uint16, same mapping)
Byte [7]    temperature + error flags
```

Enter MIT mode by sending the special frame: `{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC }` to the motor's CAN ID.

Exit MIT mode: `{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD }`.

Zero position: `{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE }`.

---

## Simulation Architecture

The simulation mirrors the real system using the same Python control code. A `SimMotorInterface` class exposes the same API as the real `SerialMotorInterface`, so all gait and kinematics code is hardware-agnostic.

```
src/control/gait_controller.py
    └── uses: MotorInterface (abstract)
              ├── SerialMotorInterface  ← real hardware path
              └── SimMotorInterface     ← MuJoCo path
```

The URDF/MJCF model lives in `src/simulation/models/`. All physical parameters (link lengths, masses, inertias) must match the mechanical team's CAD.

---

## Key Design Decisions

See `docs/decisions/` for Architecture Decision Records (ADRs).

| ADR | Decision |
|---|---|
| ADR-001 | Use PlatformIO + ESP32 for motor control |
| ADR-002 | Use MuJoCo for simulation |
| ADR-003 | Wire protocol between PC and ESP32 (TBD) |
