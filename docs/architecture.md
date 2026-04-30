# System Architecture

This document describes the software and firmware architecture of the Dal Robotics quadruped. It is the authoritative reference for how the system is structured and why.

---

## Overview

The system is divided into two compute domains:

| Domain | Hardware | Language | Responsibility |
|---|---|---|---|
| On-robot (embedded) | STM32F446RE | C++ | Real-time motor control, CAN bus communication |
| Off-robot (PC) | Windows PC | Python | Gait planning, simulation, high-level commands |

These two domains communicate over **USB Serial** during development. Wireless (WiFi via UART-attached module) is planned for Phase 5+.

---

## Layer Diagram

```
+------------------------------------------------------+
|  LAYER 5: Autonomy & AI (ML Team)                    |
|  Person detection, campus navigation, conversation   |
|  Interface: see docs/ml_interface_agreement.md       |
+------------------------------------------------------+
|  LAYER 4: High-Level Control (PC / Python)           |
|  Gait scheduler, foot trajectory planner,            |
|  body pose controller, command dispatcher            |
+------------------------------------------------------+
|  LAYER 3: Kinematics (PC / Python)                   |
|  Forward kinematics (FK), Inverse kinematics (IK)   |
|  for all 4 legs                                      |
+------------------------------------------------------+
|  LAYER 2: Serial Protocol (USB Serial / WiFi UDP)    |
|  ASCII command packets: joint targets -> STM32       |
|  Joint states, IMU data <- STM32                     |
+------------------------------------------------------+
|  LAYER 1: STM32F446RE Firmware (C++)                 |
|  Receives joint targets, encodes MIT CAN frames,     |
|  reads back motor state, safety watchdog             |
+------------------------------------------------------+
|  LAYER 0: CAN Bus + Motors                           |
|  AK40-10 x 12, MIT Mini Cheetah CAN protocol        |
+------------------------------------------------------+
```

---

## Communication Protocol (PC to STM32)

The PC sends ASCII commands over USB Serial. This keeps the protocol debuggable with any terminal. A binary format may be adopted later for deployed performance (see ADR-003).

### Command (PC -> STM32)

```
CMD <id> <pos> <vel> <kp> <kd> <torque_ff>\n
ENTER <id>\n
EXIT <id>\n
ZERO <id>\n
```

- `id`: motor CAN ID (1-12)
- `pos`: target position in radians
- `vel`: target velocity in rad/s
- `kp`: position gain in Nm/rad (0-500)
- `kd`: damping gain in Nm-s/rad (0-5)
- `torque_ff`: feedforward torque in Nm (-18 to +18)

### Response (STM32 -> PC)

```
STATE <id> <pos> <vel> <torque>\n
[ok] ...\n
[error] ...\n
```

All values are space-separated ASCII floats. State is sent in response to each CMD.

---

## CAN Bus Architecture

Each AK40-10 motor has a unique CAN ID (1-12). The STM32F446RE uses its built-in **bxCAN** peripheral (CAN 2.0B compatible). An external CAN transceiver (e.g. SN65HVD230) is required between the STM32 and the CAN bus wires -- the Electrical team provides this.

**CAN1 pins on Nucleo F446RE:** RX = PB8, TX = PB9

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

### Command frame (host -> motor)

```
Byte [0:1]  position target     (uint16, maps to -12.5 to +12.5 rad)
Byte [2]    velocity [11:4]     (upper 8 bits of 12-bit velocity)
Byte [3]    vel [3:0] | kp[11:8]
Byte [4]    kp [7:0]            (12-bit kp, maps to 0 to 500 Nm/rad)
Byte [5]    kd [11:4]           (upper 8 bits of 12-bit kd)
Byte [6]    kd [3:0] | t[11:8]  (kd maps to 0 to 5 Nm-s/rad)
Byte [7]    torque_ff [7:0]     (12-bit torque, maps to -18 to +18 Nm)
```

### Response frame (motor -> host)

```
Byte [0]    Motor ID
Byte [1:2]  position  (uint16, same mapping as command)
Byte [3:4]  velocity  (12-bit, upper 8 bits in [3], lower 4 in [4][7:4])
Byte [4:5]  torque    (12-bit, lower 4 of [4] are upper bits)
Byte [6:7]  temperature and error flags (not decoded in Phase 1)
```

Special frames (sent to motor CAN ID):

| Frame | Last byte | Purpose |
|---|---|---|
| `FF FF FF FF FF FF FF FC` | 0xFC | Enter MIT mode |
| `FF FF FF FF FF FF FF FD` | 0xFD | Exit MIT mode |
| `FF FF FF FF FF FF FF FE` | 0xFE | Set zero position |

---

## Simulation Architecture

The simulation mirrors the real system using the same Python control code. A `SimMotorInterface` class exposes the same API as the real `SerialMotorInterface`, so all gait and kinematics code is hardware-agnostic.

```
src/control/gait_controller.py
    uses: MotorInterface (abstract)
          -- SerialMotorInterface  (real STM32 path over USB Serial)
          -- SimMotorInterface     (MuJoCo path)
```

The MJCF model lives in `src/simulation/models/`. All physical parameters (link lengths, masses, inertias) must match the mechanical team's CAD.

---

## Key Design Decisions

See `docs/decisions/` for Architecture Decision Records (ADRs).

| ADR | Decision |
|---|---|
| ADR-001 | Use STM32F446RE + PlatformIO for motor control firmware |
| ADR-002 | Use MuJoCo for simulation |
| ADR-003 | Wire protocol between PC and STM32 (TBD) |
