#include "ak40_driver.h"

// Special command bytes
static const uint8_t ENTER_MODE_FRAME[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t EXIT_MODE_FRAME[8]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t SET_ZERO_FRAME[8]   = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

bool AK40Driver::begin(gpio_num_t tx_pin, gpio_num_t rx_pin) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
    if (twai_start() != ESP_OK) return false;
    return true;
}

static void send_raw(uint8_t id, const uint8_t* data) {
    twai_message_t msg;
    msg.identifier     = id;
    msg.extd           = 0;
    msg.rtr            = 0;
    msg.data_length_code = 8;
    memcpy(msg.data, data, 8);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
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
    uint8_t data[8];
    uint16_t p  = float_to_uint(pos,      P_MIN,  P_MAX,  16);
    uint16_t v  = float_to_uint(vel,      V_MIN,  V_MAX,  12);
    uint16_t kp_ = float_to_uint(kp,     KP_MIN, KP_MAX, 12);
    uint16_t kd_ = float_to_uint(kd,     KD_MIN, KD_MAX, 12);
    uint16_t t  = float_to_uint(torque_ff, T_MIN, T_MAX, 12);

    data[0] = p >> 8;
    data[1] = p & 0xFF;
    data[2] = v >> 4;
    data[3] = ((v & 0xF) << 4) | (kp_ >> 8);
    data[4] = kp_ & 0xFF;
    data[5] = kd_ >> 4;
    data[6] = ((kd_ & 0xF) << 4) | (t >> 8);
    data[7] = t & 0xFF;

    send_raw(id, data);

    // Wait for reply
    twai_message_t reply;
    if (twai_receive(&reply, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) return false;
    if (reply.data_length_code < 6) return false;

    out->id       = reply.data[0];
    uint16_t p_r  = (reply.data[1] << 8) | reply.data[2];
    uint16_t v_r  = (reply.data[3] << 4) | (reply.data[4] >> 4);
    uint16_t t_r  = ((reply.data[4] & 0xF) << 8) | reply.data[5];
    out->position = uint_to_float(p_r, P_MIN, P_MAX, 16);
    out->velocity = uint_to_float(v_r, V_MIN, V_MAX, 12);
    out->torque   = uint_to_float(t_r, T_MIN, T_MAX, 12);
    return true;
}

uint16_t AK40Driver::float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float AK40Driver::uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return (float)x * span / ((float)((1 << bits) - 1)) + x_min;
}
