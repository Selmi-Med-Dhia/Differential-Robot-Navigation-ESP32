#include "encoder_pcnt.hpp"

#include <algorithm>

#include "diffnav.hpp"
#include "robot_config.hpp"

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5

bool EncoderPcnt::configureUnit(int phase_a_pin,
                                int phase_b_pin,
                                pcnt_unit_handle_t* unit,
                                pcnt_channel_handle_t* channel_a,
                                pcnt_channel_handle_t* channel_b) {
    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = 30000;
    unit_config.low_limit = -30000;
    unit_config.flags.accum_count = 1;

    if (pcnt_new_unit(&unit_config, unit) != ESP_OK) {
        return false;
    }

    pcnt_chan_config_t channel_a_config = {};
    channel_a_config.edge_gpio_num = phase_a_pin;
    channel_a_config.level_gpio_num = phase_b_pin;

    if (pcnt_new_channel(*unit, &channel_a_config, channel_a) != ESP_OK) {
        return false;
    }
    if (pcnt_channel_set_edge_action(*channel_a,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE) != ESP_OK) {
        return false;
    }
    if (pcnt_channel_set_level_action(*channel_a,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) {
        return false;
    }

    pcnt_chan_config_t channel_b_config = {};
    channel_b_config.edge_gpio_num = phase_b_pin;
    channel_b_config.level_gpio_num = phase_a_pin;

    if (pcnt_new_channel(*unit, &channel_b_config, channel_b) != ESP_OK) {
        return false;
    }
    if (pcnt_channel_set_edge_action(*channel_b,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE) != ESP_OK) {
        return false;
    }
    if (pcnt_channel_set_level_action(*channel_b,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) {
        return false;
    }

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = robot_config::kEncoderGlitchFilterNs;

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
    return configureUnit(robot_config::kRightEncoderAPin,
                         robot_config::kRightEncoderBPin,
                         &right_unit_,
                         &right_channel_a_,
                         &right_channel_b_) &&
           configureUnit(robot_config::kLeftEncoderAPin,
                         robot_config::kLeftEncoderBPin,
                         &left_unit_,
                         &left_channel_a_,
                         &left_channel_b_);
}

EncoderCounts EncoderPcnt::snapshot() {
    int right_raw = 0;
    int left_raw = 0;

    if (right_unit_ != nullptr) {
        pcnt_unit_get_count(right_unit_, &right_raw);
    }
    if (left_unit_ != nullptr) {
        pcnt_unit_get_count(left_unit_, &left_raw);
    }

    return {
        static_cast<int64_t>(robot_config::kRightEncoderSign) * right_raw,
        static_cast<int64_t>(robot_config::kLeftEncoderSign) * left_raw,
    };
}

void EncoderPcnt::clear() {
    if (right_unit_ != nullptr) {
        pcnt_unit_clear_count(right_unit_);
    }
    if (left_unit_ != nullptr) {
        pcnt_unit_clear_count(left_unit_);
    }
}

#else

bool EncoderPcnt::configureUnit(pcnt_unit_t unit, int phase_a_pin, int phase_b_pin) {
    pcnt_config_t channel_a_config = {};
    channel_a_config.pulse_gpio_num = phase_a_pin;
    channel_a_config.ctrl_gpio_num = phase_b_pin;
    channel_a_config.unit = unit;
    channel_a_config.channel = PCNT_CHANNEL_0;
    channel_a_config.pos_mode = PCNT_COUNT_INC;
    channel_a_config.neg_mode = PCNT_COUNT_DEC;
    channel_a_config.lctrl_mode = PCNT_MODE_KEEP;
    channel_a_config.hctrl_mode = PCNT_MODE_REVERSE;
    channel_a_config.counter_h_lim = 30000;
    channel_a_config.counter_l_lim = -30000;

    if (pcnt_unit_config(&channel_a_config) != ESP_OK) {
        return false;
    }

    pcnt_config_t channel_b_config = {};
    channel_b_config.pulse_gpio_num = phase_b_pin;
    channel_b_config.ctrl_gpio_num = phase_a_pin;
    channel_b_config.unit = unit;
    channel_b_config.channel = PCNT_CHANNEL_1;
    channel_b_config.pos_mode = PCNT_COUNT_DEC;
    channel_b_config.neg_mode = PCNT_COUNT_INC;
    channel_b_config.lctrl_mode = PCNT_MODE_KEEP;
    channel_b_config.hctrl_mode = PCNT_MODE_REVERSE;
    channel_b_config.counter_h_lim = 30000;
    channel_b_config.counter_l_lim = -30000;

    if (pcnt_unit_config(&channel_b_config) != ESP_OK) {
        return false;
    }

    uint32_t filter_cycles = (robot_config::kEncoderGlitchFilterNs * 80U + 999U) / 1000U;
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
    right_total_ = 0;
    left_total_ = 0;
    previous_right_raw_ = 0;
    previous_left_raw_ = 0;

    return configureUnit(PCNT_UNIT_0,
                         robot_config::kRightEncoderAPin,
                         robot_config::kRightEncoderBPin) &&
           configureUnit(PCNT_UNIT_1,
                         robot_config::kLeftEncoderAPin,
                         robot_config::kLeftEncoderBPin);
}

EncoderCounts EncoderPcnt::snapshot() {
    int16_t right_raw = 0;
    int16_t left_raw = 0;

    pcnt_get_counter_value(PCNT_UNIT_0, &right_raw);
    pcnt_get_counter_value(PCNT_UNIT_1, &left_raw);

    const int32_t right_delta =
        diffnav::extendLegacyPcntDelta(previous_right_raw_, right_raw);
    const int32_t left_delta =
        diffnav::extendLegacyPcntDelta(previous_left_raw_, left_raw);

    right_total_ += static_cast<int64_t>(robot_config::kRightEncoderSign) * right_delta;
    left_total_ += static_cast<int64_t>(robot_config::kLeftEncoderSign) * left_delta;

    previous_right_raw_ = right_raw;
    previous_left_raw_ = left_raw;

    return {right_total_, left_total_};
}

void EncoderPcnt::clear() {
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    right_total_ = 0;
    left_total_ = 0;
    previous_right_raw_ = 0;
    previous_left_raw_ = 0;
}

#endif
