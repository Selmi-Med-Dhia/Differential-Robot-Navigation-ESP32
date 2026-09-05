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
    int64_t encoder_count_R = 0;
    int64_t encoder_count_L = 0;
};

// Hardware quadrature encoder reader. PCNT handles encoder edges in hardware, so the CPU
// does not execute an interrupt for every encoder transition.
class EncoderPcnt {
public:
    bool begin();
    EncoderCounts snapshot();
    void clear();

private:
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
    bool configureUnit(int phase_A_pin,
                       int phase_B_pin,
                       pcnt_unit_handle_t* unit,
                       pcnt_channel_handle_t* channel_A,
                       pcnt_channel_handle_t* channel_B);

    pcnt_unit_handle_t pcnt_unit_R_ = nullptr;
    pcnt_unit_handle_t pcnt_unit_L_ = nullptr;
    pcnt_channel_handle_t pcnt_channel_R_A_ = nullptr;
    pcnt_channel_handle_t pcnt_channel_R_B_ = nullptr;
    pcnt_channel_handle_t pcnt_channel_L_A_ = nullptr;
    pcnt_channel_handle_t pcnt_channel_L_B_ = nullptr;
#else
    bool configureUnit(pcnt_unit_t unit, int phase_A_pin, int phase_B_pin);

    int64_t encoder_total_R_ = 0;
    int64_t encoder_total_L_ = 0;
    int16_t previous_pcnt_R_ = 0;
    int16_t previous_pcnt_L_ = 0;
#endif
};
