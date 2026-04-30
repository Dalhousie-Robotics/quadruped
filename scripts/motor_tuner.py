#!/usr/bin/env python3
"""
motor_tuner.py -- Interactive terminal interface for AK40-10 motor control.

Connects to the STM32F446RE over USB Serial and lets you command individual
motors, adjust kp/kd gains live, and watch motor state feedback in real time.

Usage:
    python motor_tuner.py              # auto-detect STM32 COM port
    python motor_tuner.py COM3         # specify port explicitly

Requirements:
    pip install pyserial

Motor state is printed as it arrives from the STM32 in the background.
All position values are in radians. Velocity in rad/s. Torque in Nm.
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports

# -----------------------------------------------------------------------
# Default gains -- safe starting values for a benchtop single-motor test.
# Increase kp for stiffer position holding. Increase kd to damp oscillation.
# AK40-10 ranges: kp 0-500 Nm/rad, kd 0-5 Nm-s/rad
# -----------------------------------------------------------------------
DEFAULT_KP = 5.0
DEFAULT_KD = 0.5

HELP_TEXT = """
Commands
--------
  motor <id>               Select active motor (1-12)
  enter                    Enter MIT mode on active motor
  exit                     Exit MIT mode on active motor
  zero                     Set zero position (writes to motor flash -- use carefully)
  goto <pos>               Move to position [rad]
  goto <pos> <vel>         Move to position at velocity [rad/s]
  kp <value>               Set position gain (0 - 500 Nm/rad)
  kd <value>               Set damping gain (0 - 5 Nm-s/rad)
  gains                    Print current kp and kd
  state                    Print last received motor state
  hold                     Re-send last position command
  off                      Release motor (zero torque, zero gains)
  help                     Show this message
  quit                     Exit MIT mode and disconnect
"""


class MotorTuner:
    """Manages the serial link to the STM32 and motor command state."""

    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.motor_id: int = 1
        self.kp: float = DEFAULT_KP
        self.kd: float = DEFAULT_KD
        self._last_state: dict[int, tuple[float, float, float]] = {}
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _send(self, text: str) -> None:
        self.ser.write((text + "\n").encode())

    def _read_loop(self) -> None:
        """Background thread: reads lines from STM32 and stores state."""
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                if line.startswith("STATE "):
                    parts = line.split()
                    if len(parts) == 5:
                        mid = int(parts[1])
                        with self._lock:
                            self._last_state[mid] = (
                                float(parts[2]),
                                float(parts[3]),
                                float(parts[4]),
                            )
                else:
                    # Firmware log lines ([ok], [error], [boot]) go straight to console
                    print(f"\r  {line}")
                    print(f"\rmotor{self.motor_id}> ", end="", flush=True)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Motor commands
    # ------------------------------------------------------------------

    def enter_mode(self, mid: int | None = None) -> None:
        self._send(f"ENTER {mid or self.motor_id}")

    def exit_mode(self, mid: int | None = None) -> None:
        self._send(f"EXIT {mid or self.motor_id}")

    def set_zero(self, mid: int | None = None) -> None:
        self._send(f"ZERO {mid or self.motor_id}")

    def send_command(
        self,
        pos: float,
        vel: float = 0.0,
        torque_ff: float = 0.0,
        mid: int | None = None,
    ) -> None:
        m = mid or self.motor_id
        self._send(
            f"CMD {m} {pos:.4f} {vel:.4f} {self.kp:.4f} {self.kd:.4f} {torque_ff:.4f}"
        )

    # ------------------------------------------------------------------
    # State display
    # ------------------------------------------------------------------

    def print_state(self) -> None:
        with self._lock:
            entry = self._last_state.get(self.motor_id)
        if entry:
            pos, vel, torque = entry
            print(
                f"  motor {self.motor_id}: "
                f"pos={pos:+.4f} rad  "
                f"vel={vel:+.4f} rad/s  "
                f"torque={torque:+.4f} Nm"
            )
        else:
            print(f"  motor {self.motor_id}: no state received yet")

    def print_gains(self) -> None:
        print(f"  kp = {self.kp}  kd = {self.kd}")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self) -> None:
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


# -----------------------------------------------------------------------
# Port detection
# -----------------------------------------------------------------------

def find_stm32_port() -> str:
    """Return the first likely STM32 COM port, or ask the user to pick one."""
    ports = list(serial.tools.list_ports.comports())
    stm_keywords = ("STM", "ST-Link", "STMicro", "Virtual COM")
    stm_ports = [
        p for p in ports
        if any(kw.lower() in p.description.lower() for kw in stm_keywords)
    ]
    if stm_ports:
        print(f"Found STM32 on {stm_ports[0].device} ({stm_ports[0].description})")
        return stm_ports[0].device

    if not ports:
        raise RuntimeError(
            "No serial ports found. Is the STM32 plugged in via USB?"
        )

    print("No STM32 port auto-detected. Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    idx = int(input("Select port number: ").strip())
    return ports[idx].device


# -----------------------------------------------------------------------
# Main REPL
# -----------------------------------------------------------------------

def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else find_stm32_port()

    print(f"Connecting to {port} at 115200 baud...")
    tuner = MotorTuner(port)
    time.sleep(2.5)  # wait for STM32 boot messages

    print(f"Ready.  Active motor: {tuner.motor_id}  kp={tuner.kp}  kd={tuner.kd}")
    print("Type 'help' for a list of commands.\n")

    last_pos: float = 0.0

    try:
        while True:
            try:
                raw = input(f"motor{tuner.motor_id}> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            # --- quit ---
            if cmd == "quit":
                print(f"Sending exit mode to motor {tuner.motor_id}...")
                tuner.exit_mode()
                time.sleep(0.2)
                break

            # --- help ---
            elif cmd == "help":
                print(HELP_TEXT)

            # --- motor <id> ---
            elif cmd == "motor":
                if len(parts) < 2:
                    print("  Usage: motor <id>")
                else:
                    mid = int(parts[1])
                    if 1 <= mid <= 12:
                        tuner.motor_id = mid
                        print(f"  Active motor: {mid}")
                    else:
                        print("  Motor ID must be 1-12")

            # --- enter ---
            elif cmd == "enter":
                tuner.enter_mode()
                print(f"  Enter mode sent to motor {tuner.motor_id}")

            # --- exit ---
            elif cmd == "exit":
                tuner.exit_mode()
                print(f"  Exit mode sent to motor {tuner.motor_id}")

            # --- zero ---
            elif cmd == "zero":
                confirm = input(
                    "  This writes to motor flash and shifts the zero reference.\n"
                    "  Type 'yes' to confirm: "
                ).strip()
                if confirm == "yes":
                    tuner.set_zero()
                    print("  Zero position set.")
                else:
                    print("  Cancelled.")

            # --- goto <pos> [vel] ---
            elif cmd == "goto":
                if len(parts) < 2:
                    print("  Usage: goto <pos_rad> [vel_rad_s]")
                else:
                    pos = float(parts[1])
                    vel = float(parts[2]) if len(parts) > 2 else 0.0
                    last_pos = pos
                    tuner.send_command(pos, vel)
                    time.sleep(0.08)
                    tuner.print_state()

            # --- kp <value> ---
            elif cmd == "kp":
                if len(parts) < 2:
                    print(f"  kp = {tuner.kp}  (range 0 - 500 Nm/rad)")
                else:
                    val = float(parts[1])
                    if 0.0 <= val <= 500.0:
                        tuner.kp = val
                        print(f"  kp = {tuner.kp}")
                    else:
                        print("  kp must be between 0 and 500")

            # --- kd <value> ---
            elif cmd == "kd":
                if len(parts) < 2:
                    print(f"  kd = {tuner.kd}  (range 0 - 5 Nm-s/rad)")
                else:
                    val = float(parts[1])
                    if 0.0 <= val <= 5.0:
                        tuner.kd = val
                        print(f"  kd = {tuner.kd}")
                    else:
                        print("  kd must be between 0 and 5")

            # --- gains ---
            elif cmd == "gains":
                tuner.print_gains()

            # --- state ---
            elif cmd == "state":
                tuner.print_state()

            # --- hold ---
            elif cmd == "hold":
                tuner.send_command(last_pos)
                time.sleep(0.08)
                tuner.print_state()

            # --- off ---
            elif cmd == "off":
                tuner.kp = 0.0
                tuner.kd = 0.0
                tuner.send_command(last_pos, 0.0, 0.0)
                print("  Motor released (kp=0, kd=0, torque=0). Set gains to re-engage.")

            else:
                print(f"  Unknown command '{cmd}'. Type 'help'.")

    finally:
        tuner.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
