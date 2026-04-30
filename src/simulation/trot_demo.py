"""
trot_demo.py -- Open-loop trot gait demonstration.

Diagonal leg pairs move together (FL+RR, FR+RL). Each leg follows a
sinusoidal hip trajectory with a knee lift during the swing phase.
This is an open-loop preview of Phase 4 -- no balance feedback yet,
so the robot may drift or fall on a long run.

Usage:
    python src/simulation/trot_demo.py

Controls:
    Space       Pause / unpause
    Backspace   Reset
    Esc         Quit
"""

import os
import time
import numpy as np
import mujoco
import mujoco.viewer

MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "quadruped.xml")

# ---------------------------------------------------------------------------
# Gait parameters -- tune these to change the walk
# ---------------------------------------------------------------------------
GAIT_FREQ      = 1.4    # Hz  -- step frequency (higher = faster steps)
HIP_FE_CENTER  = 0.50   # rad -- nominal hip angle (forward lean)
HIP_FE_AMP     = 0.26   # rad -- how far the hip swings each step
KNEE_STANCE    = -1.30  # rad -- knee angle when foot is on the ground
KNEE_SWING     = -1.85  # rad -- knee angle at peak of swing (foot lift height)
HIP_AB         = 0.0    # rad -- lateral hip (0 = straight)

PD_KP          = 45.0   # Nm/rad -- position gain
PD_KD          = 1.5    # Nm-s/rad -- damping gain

WARMUP_SECS    = 0.8    # seconds to stand still before walking starts

# ---------------------------------------------------------------------------
# Trot phase offsets  (diagonal pairs share the same phase)
#   FL + RR = pair A    (phase offset 0)
#   FR + RL = pair B    (phase offset pi = half cycle behind)
# ---------------------------------------------------------------------------
PHASE_OFFSET = {
    "FL": 0.0,
    "RR": 0.0,
    "FR": np.pi,
    "RL": np.pi,
}


def leg_targets(leg: str, gait_phase: float) -> tuple[float, float, float]:
    """
    Return (hip_ab, hip_fe, knee) targets for one leg at the given gait phase.

    The gait cycle for each leg (after adding its phase offset):
      0  -> pi   : SWING  -- foot in the air, hip moving forward
      pi -> 2*pi : STANCE -- foot on ground, hip moving backward (propels body)
    """
    phase = (gait_phase + PHASE_OFFSET[leg]) % (2.0 * np.pi)

    # Hip fe: sinusoidal.
    #   sin > 0 (swing)   -> hip forward of center
    #   sin < 0 (stance)  -> hip behind center (body pushed forward)
    hip_fe = HIP_FE_CENTER + HIP_FE_AMP * np.sin(phase)

    # Knee: lift during swing (0 to pi), hold stance angle otherwise.
    if phase < np.pi:
        swing_factor = np.sin(phase)          # 0 -> 1 -> 0, smooth bell curve
        knee = KNEE_STANCE + (KNEE_SWING - KNEE_STANCE) * swing_factor
    else:
        knee = KNEE_STANCE

    return HIP_AB, hip_fe, knee


def pd_control(model, data, targets: dict, kp: float, kd: float) -> None:
    """Apply PD torque control toward `targets` (joint_name -> angle_rad)."""
    for joint_name, target_pos in targets.items():
        j_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    joint_name)
        a_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name + "_act")
        if j_id < 0 or a_id < 0:
            continue
        pos    = data.qpos[model.jnt_qposadr[j_id]]
        vel    = data.qvel[model.jnt_dofadr[j_id]]
        torque = kp * (target_pos - pos) - kd * vel
        data.ctrl[a_id] = float(np.clip(torque, -18.0, 18.0))


def stand_targets() -> dict:
    """Joint targets for the neutral standing pose."""
    targets = {}
    for leg in ("FL", "FR", "RL", "RR"):
        targets[f"{leg}_hip_ab"] = 0.0
        targets[f"{leg}_hip_fe"] = HIP_FE_CENTER
        targets[f"{leg}_knee"]   = KNEE_STANCE
    return targets


def main() -> None:
    print("Loading model:", MODEL_PATH)
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    # Start from the keyframe standing pose
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    print(f"Gait: trot  freq={GAIT_FREQ} Hz  hip_amp={HIP_FE_AMP} rad")
    print(f"      kp={PD_KP}  kd={PD_KD}")
    print(f"Warming up for {WARMUP_SECS}s then walking starts.")
    print("Space=pause  Backspace=reset  Esc=quit")

    sim_time  = 0.0
    dt        = model.opt.timestep

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

            if sim_time < WARMUP_SECS:
                # Hold standing pose while the robot settles
                targets = stand_targets()
            else:
                # Trot gait
                walk_time  = sim_time - WARMUP_SECS
                gait_phase = (2.0 * np.pi * GAIT_FREQ * walk_time) % (2.0 * np.pi)
                targets    = {}
                for leg in ("FL", "FR", "RL", "RR"):
                    hab, hfe, kne = leg_targets(leg, gait_phase)
                    targets[f"{leg}_hip_ab"] = hab
                    targets[f"{leg}_hip_fe"] = hfe
                    targets[f"{leg}_knee"]   = kne

            pd_control(model, data, targets, PD_KP, PD_KD)
            mujoco.mj_step(model, data)
            sim_time += dt
            viewer.sync()


if __name__ == "__main__":
    main()
