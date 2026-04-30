#include <Arduino.h>
#include "ak40_driver.h"

// USB CDC serial to PC (mapped to Serial by -DSERIAL_USB in platformio.ini)
#define PC_SERIAL Serial
#define PC_BAUD   115200

// -----------------------------------------------------------------------
// Command protocol (PC -> STM32, newline-terminated ASCII):
//
//   CMD <id> <pos> <vel> <kp> <kd> <torque_ff>
//       Send MIT control frame. Replies with STATE line.
//
//   ENTER <id>
//       Send enter-MIT-mode frame to motor.
//
//   EXIT <id>
//       Send exit-MIT-mode frame to motor.
//
//   ZERO <id>
//       Set zero position on motor (writes to motor flash).
//
// Response (STM32 -> PC):
//
//   STATE <id> <pos_rad> <vel_rad_s> <torque_Nm>
//   [error] ...
//   [ok] ...
// -----------------------------------------------------------------------

void setup() {
    PC_SERIAL.begin(PC_BAUD);

    // Give USB CDC time to enumerate on the host side
    uint32_t t = millis();
    while (!PC_SERIAL && (millis() - t) < 3000) {}

    PC_SERIAL.println("[boot] Dal Robotics Quadruped Firmware");
    PC_SERIAL.println("[boot] STM32F446RE @ 1 Mbit/s CAN");

    if (!AK40Driver::begin()) {
        PC_SERIAL.println("[error] CAN init failed -- check transceiver wiring");
        PC_SERIAL.println("[error] CAN1: RX=PB8  TX=PB9 (requires external transceiver)");
        while (true) { delay(1000); }
    }

    PC_SERIAL.println("[ok] CAN bus ready");
    PC_SERIAL.println("[ok] Waiting for commands from PC");
}

void loop() {
    if (!PC_SERIAL.available()) return;

    String line = PC_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // --- CMD <id> <pos> <vel> <kp> <kd> <torque_ff> ---
    if (line.startsWith("CMD ")) {
        uint8_t id;
        float pos, vel, kp, kd, tff;
        int n = sscanf(line.c_str(),
                       "CMD %hhu %f %f %f %f %f",
                       &id, &pos, &vel, &kp, &kd, &tff);
        if (n != 6) {
            PC_SERIAL.println("[error] CMD parse failed -- expected: CMD <id> <pos> <vel> <kp> <kd> <tff>");
            return;
        }
        MotorState state;
        bool ok = AK40Driver::send_command(id, pos, vel, kp, kd, tff, &state);
        if (ok) {
            char buf[80];
            snprintf(buf, sizeof(buf),
                     "STATE %d %.4f %.4f %.4f",
                     (int)state.id, state.position, state.velocity, state.torque);
            PC_SERIAL.println(buf);
        } else {
            PC_SERIAL.print("[error] Motor ");
            PC_SERIAL.print((int)id);
            PC_SERIAL.println(" did not respond");
        }

    // --- ENTER <id> ---
    } else if (line.startsWith("ENTER ")) {
        uint8_t id = (uint8_t)line.substring(6).toInt();
        AK40Driver::enter_mode(id);
        PC_SERIAL.print("[ok] Enter mode -> motor ");
        PC_SERIAL.println((int)id);

    // --- EXIT <id> ---
    } else if (line.startsWith("EXIT ")) {
        uint8_t id = (uint8_t)line.substring(5).toInt();
        AK40Driver::exit_mode(id);
        PC_SERIAL.print("[ok] Exit mode -> motor ");
        PC_SERIAL.println((int)id);

    // --- ZERO <id> ---
    } else if (line.startsWith("ZERO ")) {
        uint8_t id = (uint8_t)line.substring(5).toInt();
        AK40Driver::set_zero(id);
        PC_SERIAL.print("[ok] Zero set -> motor ");
        PC_SERIAL.println((int)id);

    } else {
        PC_SERIAL.print("[error] Unknown command: ");
        PC_SERIAL.println(line);
    }
}
