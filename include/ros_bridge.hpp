#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "diffnav.hpp"
struct TelemetryFrame{diffnav::OdometryState odom{};diffnav::MotionStatus motion{};diffnav::WheelSpeeds target{};float right_pwm=0,left_pwm=0;};
struct RosBridgeContext{QueueHandle_t command_queue=nullptr,telemetry_queue=nullptr;};
void rosBridgeTask(void* argument);
