#pragma once
#include <Arduino.h>
#include <driver/twai.h>

// CAN parameter ranges for AK40-10 (MIT Mini Cheetah protocol)
constexpr float P_MIN  = -12.5f;  // rad
constexpr float P_MAX  =  12.5f;
constexpr float V_MIN  = -65.0f;  // rad/s
constexpr float V_MAX  =  65.0f;
constexpr float KP_MIN =   0.0f;  // Nm/rad
constexpr float KP_MAX = 500.0f;
constexpr float KD_MIN =   0.0f;  // Nm·s/rad
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
    // Initialize TWAI peripheral. Returns false if hardware init fails.
    static bool begin(gpio_num_t tx_pin, gpio_num_t rx_pin);

    // Enter MIT mode on motor `id`. Must be called before sending commands.
    static void enter_mode(uint8_t id);

    // Exit MIT mode. Always call this on shutdown.
    static void exit_mode(uint8_t id);

    // Set motor zero position (use carefully, saves to flash on motor).
    static void set_zero(uint8_t id);

    // Send a MIT-mode command and read back state synchronously.
    // Returns false if the motor does not respond within timeout.
    static bool send_command(uint8_t id,
                             float pos, float vel,
                             float kp,  float kd,
                             float torque_ff,
                             MotorState* out,
                             uint32_t timeout_ms = 5);

private:
    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
    static float    uint_to_float(uint16_t x, float x_min, float x_max, int bits);
};
