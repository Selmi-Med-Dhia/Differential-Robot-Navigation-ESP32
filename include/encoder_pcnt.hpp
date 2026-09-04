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
struct EncoderCounts{int64_t right=0,left=0;};
class EncoderPcnt{public:bool begin();EncoderCounts snapshot();void clear();private:
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
 bool configureUnit(int a,int b,pcnt_unit_handle_t* u,pcnt_channel_handle_t* ca,pcnt_channel_handle_t* cb);pcnt_unit_handle_t ru_=nullptr,lu_=nullptr;pcnt_channel_handle_t rca_=nullptr,rcb_=nullptr,lca_=nullptr,lcb_=nullptr;
#else
 bool configureUnit(pcnt_unit_t u,int a,int b);int64_t rt_=0,lt_=0;int16_t pr_=0,pl_=0;
#endif
};
