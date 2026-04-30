#include <Arduino.h>
#include "ak40_driver.h"

// CAN transceiver pins (adjust to your wiring)
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// Serial comms with PC
#define PC_SERIAL Serial
#define PC_BAUD   115200

void setup() {
    PC_SERIAL.begin(PC_BAUD);
    PC_SERIAL.println("[boot] Dal Robotics Quadruped Firmware");

    if (!AK40Driver::begin(CAN_TX_PIN, CAN_RX_PIN)) {
        PC_SERIAL.println("[error] CAN bus init failed, check wiring");
        while (true) delay(1000);
    }

    PC_SERIAL.println("[ok] CAN bus ready");
}

void loop() {
    // Main control loop placeholder
    // TODO Phase 1: receive command from PC, send to motors, read back state
    delay(1);
}
