#include "encoder_pcnt.hpp"

#include <algorithm>

#include "diffnav.hpp"
#include "robot_config.hpp"

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5

bool EncoderPcnt::configureUnit(int phase_A_pin,
                                int phase_B_pin,
                                pcnt_unit_handle_t* unit,
                                pcnt_channel_handle_t* channel_A,
                                pcnt_channel_handle_t* channel_B) {
    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = 30000;
    unit_config.low_limit = -30000;
    unit_config.flags.accum_count = 1;

    if (pcnt_new_unit(&unit_config, unit) != ESP_OK) {
        return false;
    }

    // Channel A counts both A edges and uses B level to determine direction.
    pcnt_chan_config_t channel_A_config = {};
    channel_A_config.edge_gpio_num = phase_A_pin;
    channel_A_config.level_gpio_num = phase_B_pin;
    if (pcnt_new_channel(*unit, &channel_A_config, channel_A) != ESP_OK) {
        return false;
    }
    if (pcnt_channel_set_edge_action(*channel_A,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE) != ESP_OK ||
        pcnt_channel_set_level_action(*channel_A,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) {
        return false;
    }

    // Channel B does the complementary operation. Together A+B give x4 quadrature count.
    pcnt_chan_config_t channel_B_config = {};
    channel_B_config.edge_gpio_num = phase_B_pin;
    channel_B_config.level_gpio_num = phase_A_pin;
    if (pcnt_new_channel(*unit, &channel_B_config, channel_B) != ESP_OK) {
        return false;
    }
    if (pcnt_channel_set_edge_action(*channel_B,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE) != ESP_OK ||
        pcnt_channel_set_level_action(*channel_B,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) {
        return false;
    }

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = robot_config::encoder_glitch_filter_ns;
    if (pcnt_unit_set_glitch_filter(*unit, &filter_config) != ESP_OK) {
        return false;
    }

    if (pcnt_unit_add_watch_point(*unit, unit_config.high_limit) != ESP_OK ||
        pcnt_unit_add_watch_point(*unit, unit_config.low_limit) != ESP_OK) {
        return false;
    }

    return pcnt_unit_enable(*unit) == ESP_OK &&
           pcnt_unit_clear_count(*unit) == ESP_OK &&
           pcnt_unit_start(*unit) == ESP_OK;
}

bool EncoderPcnt::begin() {
    return configureUnit(robot_config::encoder_R_A_pin,
                         robot_config::encoder_R_B_pin,
                         &pcnt_unit_R_,
                         &pcnt_channel_R_A_,
                         &pcnt_channel_R_B_) &&
           configureUnit(robot_config::encoder_L_A_pin,
                         robot_config::encoder_L_B_pin,
                         &pcnt_unit_L_,
                         &pcnt_channel_L_A_,
                         &pcnt_channel_L_B_);
}

EncoderCounts EncoderPcnt::snapshot() {
    int raw_count_R = 0;
    int raw_count_L = 0;

    if (pcnt_unit_R_ != nullptr) {
        pcnt_unit_get_count(pcnt_unit_R_, &raw_count_R);
    }
    if (pcnt_unit_L_ != nullptr) {
        pcnt_unit_get_count(pcnt_unit_L_, &raw_count_L);
    }

    return {
        static_cast<int64_t>(robot_config::encoder_direction_R) * raw_count_R,
        static_cast<int64_t>(robot_config::encoder_direction_L) * raw_count_L,
    };
}

void EncoderPcnt::clear() {
    if (pcnt_unit_R_ != nullptr) {
        pcnt_unit_clear_count(pcnt_unit_R_);
    }
    if (pcnt_unit_L_ != nullptr) {
        pcnt_unit_clear_count(pcnt_unit_L_);
    }
}

#else

bool EncoderPcnt::configureUnit(pcnt_unit_t unit, int phase_A_pin, int phase_B_pin) {
    // Legacy ESP32 PCNT has two channels per unit. Using both channels gives x4 quadrature
    // without a GPIO ISR on each encoder edge.
    pcnt_config_t channel_A_config = {};
    channel_A_config.pulse_gpio_num = phase_A_pin;
    channel_A_config.ctrl_gpio_num = phase_B_pin;
    channel_A_config.unit = unit;
    channel_A_config.channel = PCNT_CHANNEL_0;
    channel_A_config.pos_mode = PCNT_COUNT_INC;
    channel_A_config.neg_mode = PCNT_COUNT_DEC;
    channel_A_config.lctrl_mode = PCNT_MODE_KEEP;
    channel_A_config.hctrl_mode = PCNT_MODE_REVERSE;
    channel_A_config.counter_h_lim = 30000;
    channel_A_config.counter_l_lim = -30000;

    if (pcnt_unit_config(&channel_A_config) != ESP_OK) {
        return false;
    }

    pcnt_config_t channel_B_config = {};
    channel_B_config.pulse_gpio_num = phase_B_pin;
    channel_B_config.ctrl_gpio_num = phase_A_pin;
    channel_B_config.unit = unit;
    channel_B_config.channel = PCNT_CHANNEL_1;
    channel_B_config.pos_mode = PCNT_COUNT_DEC;
    channel_B_config.neg_mode = PCNT_COUNT_INC;
    channel_B_config.lctrl_mode = PCNT_MODE_KEEP;
    channel_B_config.hctrl_mode = PCNT_MODE_REVERSE;
    channel_B_config.counter_h_lim = 30000;
    channel_B_config.counter_l_lim = -30000;

    if (pcnt_unit_config(&channel_B_config) != ESP_OK) {
        return false;
    }

    // The legacy filter is expressed in APB clock cycles (80 MHz on classic ESP32).
    uint32_t filter_cycles = (robot_config::encoder_glitch_filter_ns * 80U + 999U) / 1000U;
    filter_cycles = std::max<uint32_t>(1, std::min<uint32_t>(1023, filter_cycles));

    if (pcnt_set_filter_value(unit, static_cast<uint16_t>(filter_cycles)) != ESP_OK ||
        pcnt_filter_enable(unit) != ESP_OK) {
        return false;
    }

    return pcnt_counter_pause(unit) == ESP_OK &&
           pcnt_counter_clear(unit) == ESP_OK &&
           pcnt_counter_resume(unit) == ESP_OK;
}

bool EncoderPcnt::begin() {
    encoder_total_R_ = 0;
    encoder_total_L_ = 0;
    previous_pcnt_R_ = 0;
    previous_pcnt_L_ = 0;

    return configureUnit(PCNT_UNIT_0,
                         robot_config::encoder_R_A_pin,
                         robot_config::encoder_R_B_pin) &&
           configureUnit(PCNT_UNIT_1,
                         robot_config::encoder_L_A_pin,
                         robot_config::encoder_L_B_pin);
}

EncoderCounts EncoderPcnt::snapshot() {
    int16_t raw_count_R = 0;
    int16_t raw_count_L = 0;

    pcnt_get_counter_value(PCNT_UNIT_0, &raw_count_R);
    pcnt_get_counter_value(PCNT_UNIT_1, &raw_count_L);

    // Never clear the hardware counter in the normal control loop. Reading then clearing can
    // lose an edge that arrives between those operations. Instead we reconstruct the delta
    // when the PCNT hardware crosses its configured limit.
    const int32_t delta_R = diffnav::extendPcntDelta(previous_pcnt_R_, raw_count_R);
    const int32_t delta_L = diffnav::extendPcntDelta(previous_pcnt_L_, raw_count_L);

    encoder_total_R_ += static_cast<int64_t>(robot_config::encoder_direction_R) * delta_R;
    encoder_total_L_ += static_cast<int64_t>(robot_config::encoder_direction_L) * delta_L;

    previous_pcnt_R_ = raw_count_R;
    previous_pcnt_L_ = raw_count_L;

    return {encoder_total_R_, encoder_total_L_};
}

void EncoderPcnt::clear() {
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    encoder_total_R_ = 0;
    encoder_total_L_ = 0;
    previous_pcnt_R_ = 0;
    previous_pcnt_L_ = 0;
}

#endif
