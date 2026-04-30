#include "ak40_driver.h"

// Special command frames (MIT protocol)
static const uint8_t ENTER_MODE_FRAME[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t EXIT_MODE_FRAME[8]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t SET_ZERO_FRAME[8]   = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

bool AK40Driver::begin() {
    // CAN_BAUD_RATE is defined in platformio.ini build_flags (1000000 = 1 Mbit/s)
    return CAN.begin(CAN_BAUD_RATE);
}

static void send_raw(uint8_t id, const uint8_t* data) {
    CAN.beginPacket(id);
    CAN.write(data, 8);
    CAN.endPacket();
}

void AK40Driver::enter_mode(uint8_t id) { send_raw(id, ENTER_MODE_FRAME); }
void AK40Driver::exit_mode(uint8_t id)  { send_raw(id, EXIT_MODE_FRAME);  }
void AK40Driver::set_zero(uint8_t id)   { send_raw(id, SET_ZERO_FRAME);   }

bool AK40Driver::send_command(uint8_t id,
                               float pos, float vel,
                               float kp,  float kd,
                               float torque_ff,
                               MotorState* out,
                               uint32_t timeout_ms) {
    // Encode MIT command frame
    uint8_t data[8];
    uint16_t p   = float_to_uint(pos,       P_MIN,  P_MAX,  16);
    uint16_t v   = float_to_uint(vel,       V_MIN,  V_MAX,  12);
    uint16_t kp_ = float_to_uint(kp,        KP_MIN, KP_MAX, 12);
    uint16_t kd_ = float_to_uint(kd,        KD_MIN, KD_MAX, 12);
    uint16_t t   = float_to_uint(torque_ff, T_MIN,  T_MAX,  12);

    data[0] = p >> 8;
    data[1] = p & 0xFF;
    data[2] = v >> 4;
    data[3] = ((v & 0xF) << 4) | (kp_ >> 8);
    data[4] = kp_ & 0xFF;
    data[5] = kd_ >> 4;
    data[6] = ((kd_ & 0xF) << 4) | (t >> 8);
    data[7] = t & 0xFF;

    send_raw(id, data);

    // Poll for reply
    uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        int size = CAN.parsePacket();
        if (size >= 6) {
            uint8_t buf[8] = {};
            for (int i = 0; i < size && i < 8; i++) {
                buf[i] = (uint8_t)CAN.read();
            }
            out->id       = buf[0];
            uint16_t p_r  = ((uint16_t)buf[1] << 8) | buf[2];
            uint16_t v_r  = ((uint16_t)buf[3] << 4) | (buf[4] >> 4);
            uint16_t t_r  = (((uint16_t)buf[4] & 0xF) << 8) | buf[5];
            out->position = uint_to_float(p_r, P_MIN, P_MAX, 16);
            out->velocity = uint_to_float(v_r, V_MIN, V_MAX, 12);
            out->torque   = uint_to_float(t_r, T_MIN, T_MAX, 12);
            return true;
        }
    }
    return false;
}

uint16_t AK40Driver::float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * (float)((1 << bits) - 1) / span);
}

float AK40Driver::uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return (float)x * span / (float)((1 << bits) - 1) + x_min;
}
