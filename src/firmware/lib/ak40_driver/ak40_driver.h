#pragma once
#include <Arduino.h>
#include <CAN.h>

// CAN parameter ranges for AK40-10 (MIT Mini Cheetah protocol)
constexpr float P_MIN  = -12.5f;  // rad
constexpr float P_MAX  =  12.5f;
constexpr float V_MIN  = -65.0f;  // rad/s
constexpr float V_MAX  =  65.0f;
constexpr float KP_MIN =   0.0f;  // Nm/rad
constexpr float KP_MAX = 500.0f;
constexpr float KD_MIN =   0.0f;  // Nm-s/rad
constexpr float KD_MAX =   5.0f;
constexpr float T_MIN  = -18.0f;  // Nm
constexpr float T_MAX  =  18.0f;

struct MotorState {
    uint8_t id;
    float   position;  // rad
    float   velocity;  // rad/s
    float   torque;    // Nm
};

class AK40Driver {
public:
    // Initialize the STM32 bxCAN peripheral at CAN_BAUD_RATE.
    // CAN1 pins on STM32F446RE: RX = PB8, TX = PB9.
    // Returns false if the peripheral fails to start.
    static bool begin();

    // Enter MIT mode on motor `id`. Must be called before sending commands.
    static void enter_mode(uint8_t id);

    // Exit MIT mode. Always call this on shutdown.
    static void exit_mode(uint8_t id);

    // Set motor zero position (use carefully -- saves to flash on motor).
    static void set_zero(uint8_t id);

    // Send a MIT-mode command and read back state.
    // Polls for a reply until timeout_ms elapses.
    // Returns false if the motor does not respond in time.
    static bool send_command(uint8_t id,
                             float pos, float vel,
                             float kp,  float kd,
                             float torque_ff,
                             MotorState* out,
                             uint32_t timeout_ms = 10);

private:
    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
    static float    uint_to_float(uint16_t x, float x_min, float x_max, int bits);
};
