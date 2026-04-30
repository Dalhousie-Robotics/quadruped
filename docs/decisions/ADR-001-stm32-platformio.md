# ADR-001: Use STM32 with PlatformIO for Motor Control Firmware

**Date:** 2026-04-30
**Status:** Accepted
**Deciders:** Software Team Lead

---

## Context

We need a microcontroller to run the real-time CAN bus control loop for 12 AK40-10 motors. The controller must:
- Generate CAN frames at 500-1000 Hz without missing deadlines
- Communicate with a Windows PC over USB Serial (development) or WiFi (deployed)
- Be accessible to team members with varying experience levels
- Have reliable, well-documented CAN hardware support

## Decision

Use the **STM32F446RE** (on the ST Nucleo F446RE development board) programmed with the **PlatformIO** toolchain inside VS Code. The Arduino framework (via stm32duino) is used to reduce onboarding friction.

## Rationale

| Option | Pros | Cons |
|---|---|---|
| STM32F446RE + PlatformIO | Industry-standard real-time MCU, hardware bxCAN peripheral, 180 MHz, ST-Link on Nucleo for easy flashing, well-supported in stm32duino | No built-in WiFi (can add ESP8266/ESP32 module for wireless later) |
| ESP32 + PlatformIO | Built-in WiFi, built-in TWAI/CAN, large community | TWAI peripheral is less mature than STM32 bxCAN, 240 MHz but weaker real-time guarantees |
| STM32 + CubeIDE | Full HAL, most control | Much steeper learning curve, poor CI integration |
| Raspberry Pi | Full Linux, easy Python | Not real-time, requires separate CAN hat |

The STM32F446RE was chosen because:
- Its bxCAN peripheral is proven in automotive and robotics applications
- The Nucleo board has an on-board ST-Link debugger (flash and debug with one USB cable)
- stm32duino makes the Arduino API available, reducing the learning gap for new members
- 180 MHz Cortex-M4 with FPU is well above what the control loop needs
- WiFi can be added as a UART-attached module in a later phase without changing the CAN architecture

PlatformIO was chosen over STM32CubeIDE because it integrates with VS Code, supports `pio run` in CI, and has a clean `lib_deps` system for managing the CAN library.

## Hardware Notes

- **Board:** ST Nucleo F446RE
- **CAN peripheral:** bxCAN (CAN 2.0B), via `stm32duino/STM32duino CAN`
- **CAN pins (CAN1):** RX = PB8, TX = PB9
- **External CAN transceiver required:** e.g. SN65HVD230 or TJA1050 (Electrical team provides this)
- **PC communication:** USB CDC (`-DSERIAL_USB` flag, maps `Serial` to USB virtual COM)
- **CAN baud rate:** 1 Mbit/s (AK40-10 default)

## Consequences

- All firmware is written in C++ targeting the STM32 Arduino framework via stm32duino.
- The STM32 does not have built-in WiFi. Phase 1 and Phase 2 use USB Serial. Wireless communication is deferred to Phase 5+.
- Team members writing firmware need basic C++ knowledge.
- The `stm32duino/STM32duino CAN` library abstracts the HAL CAN API into a simple Arduino-style interface.
