# Dal Robotics — Quadruped Robot

A campus-autonomous quadruped robot with AI-driven student interaction, built by the Dal Robotics club at Dalhousie University.

---

## Quick Links

| Resource | Link |
|---|---|
| Onboarding (start here) | https://github.com/Dalhousie-Robotics/onboarding |
| Team Roadmap | https://github.com/Dalhousie-Robotics/team-docs/blob/main/roadmap.md |
| Architecture Overview | [docs/architecture.md](docs/architecture.md) |
| Contributing Guide | [CONTRIBUTING.md](CONTRIBUTING.md) |
| Meeting Notes | https://github.com/Dalhousie-Robotics/team-docs/tree/main/meetings |
| ML Interface Agreement | https://github.com/Dalhousie-Robotics/team-docs/blob/main/ml_interface_agreement.md |

---

## What We're Building

A 12-DOF quadruped robot (3 joints per leg) inspired by MIT Cheetah and Boston Dynamics Spot. The robot navigates Dalhousie's campus autonomously and interacts with students using onboard AI.

**Subsystems:**

| Subsystem | Description |
|---|---|
| Mechanical | Frame, leg linkages, body structure |
| Electrical | Power distribution, wiring, sensors |
| Software & Simulations | Motor firmware, gait control, simulation |
| Machine Learning | Person detection, navigation AI, conversation |

---

## System Architecture

```
Windows PC (Python)
  ├── Gait planner, kinematics, path commands
  └── MuJoCo simulation environment
        │
        │  USB Serial (dev) / WiFi UDP (deployed)
        ▼
ESP32 Microcontroller (C++ firmware)
  ├── Real-time CAN control loop @ 500–1000 Hz
  └── IMU reading, safety watchdog
        │
        │  CAN bus
        ▼
AK40-10 Motors × 12 (CubeMars)
  └── MIT Mini Cheetah CAN protocol
```

---

## Repository Structure

```
quadruped/
├── src/
│   ├── firmware/          ESP32 firmware (PlatformIO / C++)
│   │   ├── src/           Main firmware source
│   │   ├── include/       Header files
│   │   └── lib/           Libraries (motor driver, comms)
│   ├── control/           High-level Python control (PC-side)
│   └── simulation/        MuJoCo simulation models and scripts
├── docs/
│   ├── architecture.md    System architecture deep-dive
│   └── decisions/         Architecture Decision Records (ADRs)
├── tests/                 Unit and integration tests
├── scripts/               Utility scripts (calibration, tools)
└── .github/
    ├── workflows/         CI pipelines
    └── ISSUE_TEMPLATE/    Bug and feature request templates
```

---

## Tech Stack

| Layer | Technology |
|---|---|
| ESP32 Firmware | C++ via PlatformIO |
| CAN Communication | ESP32 TWAI peripheral + MIT protocol |
| High-Level Control | Python 3.10+ |
| Simulation | MuJoCo 3.x (Windows) |
| Motor Configuration | R-Link (CubeMars software) |
| PC ↔ ESP32 Comms | USB Serial → WiFi UDP |
| CI/CD | GitHub Actions |

---

## Getting Started

1. Follow the [Onboarding Guide](https://github.com/Dalhousie-Robotics/onboarding) — do not skip this.
2. Find your name in the team table below and pick up your assigned task.
3. Read [CONTRIBUTING.md](CONTRIBUTING.md) before opening your first PR.

---

## Team

### Firmware
*Writes C++ code that runs on the ESP32 — handles CAN bus communication with motors in real time.*

| Name | GitHub | Current Task |
|---|---|---|
| | | |
| | | |

### Simulation
*Builds and maintains the MuJoCo robot model — lets the team test algorithms without touching hardware.*

| Name | GitHub | Current Task |
|---|---|---|
| | | |

### Kinematics & Control
*Writes Python algorithms for leg movement — forward/inverse kinematics, gait planning, foot trajectories.*

| Name | GitHub | Current Task |
|---|---|---|
| | | |
| | | |

### Integration & Lead
*Maintains repo health, reviews all PRs, coordinates between areas, owns the architecture.*

| Name | GitHub | Current Task |
|---|---|---|
| | | Software Lead |

*Fill in names and tasks when roles are assigned at the first meeting.*

---

## Current Status

**Phase 0 — Infrastructure** (active)

See the full [roadmap](https://github.com/Dalhousie-Robotics/team-docs/blob/main/roadmap.md) for phase breakdown.
