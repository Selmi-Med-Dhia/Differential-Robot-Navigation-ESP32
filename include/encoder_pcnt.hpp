#pragma once

#include <Arduino.h>
#include <cstdint>

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
#include "driver/pulse_cnt.h"
#else
#include "driver/pcnt.h"
#endif

struct EncoderCounts {
    int64_t right = 0;
    int64_t left = 0;
};

class EncoderPcnt {
public:
    bool begin();
    EncoderCounts snapshot();
    void clear();

private:
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
    bool configureUnit(int phase_a_pin,
                       int phase_b_pin,
                       pcnt_unit_handle_t* unit,
                       pcnt_channel_handle_t* channel_a,
                       pcnt_channel_handle_t* channel_b);

    pcnt_unit_handle_t right_unit_ = nullptr;
    pcnt_unit_handle_t left_unit_ = nullptr;
    pcnt_channel_handle_t right_channel_a_ = nullptr;
    pcnt_channel_handle_t right_channel_b_ = nullptr;
    pcnt_channel_handle_t left_channel_a_ = nullptr;
    pcnt_channel_handle_t left_channel_b_ = nullptr;
#else
    bool configureUnit(pcnt_unit_t unit, int phase_a_pin, int phase_b_pin);

    int64_t right_total_ = 0;
    int64_t left_total_ = 0;
    int16_t previous_right_raw_ = 0;
    int16_t previous_left_raw_ = 0;
#endif
};
