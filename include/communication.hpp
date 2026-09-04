#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "diffnav.hpp"

#if defined(COMMUNICATION_MICROROS) && defined(COMMUNICATION_UART)
#error "Select only one communication backend"
#elif !defined(COMMUNICATION_MICROROS) && !defined(COMMUNICATION_UART)
#error "Select COMMUNICATION_MICROROS or COMMUNICATION_UART in platformio.ini"
#endif

struct TelemetryFrame {
    diffnav::OdometryState odometry{};
    diffnav::MotionStatus motion{};
    diffnav::WheelSpeeds target_wheel_speed{};
    float right_pwm = 0.0f;
    float left_pwm = 0.0f;
};

struct CommunicationContext {
    QueueHandle_t command_queue = nullptr;
    QueueHandle_t telemetry_queue = nullptr;
};

// Exactly one backend implements this function at compile time.
void communicationTask(void* argument);
