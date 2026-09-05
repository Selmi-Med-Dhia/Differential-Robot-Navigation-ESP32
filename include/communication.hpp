#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "diffnav.hpp"

// Information produced by the control task and consumed by either communication backend.
struct TelemetryFrame {
    diffnav::OdometryState odometry{};
    diffnav::MotionStatus motion{};
    diffnav::WheelSpeeds target_wheel_speed{};
    float pwm_R = 0.0f;
    float pwm_L = 0.0f;
};

// Both communication tasks receive the same queues. This keeps communication completely
// separate from the 200 Hz navigation/control loop.
struct CommunicationContext {
    QueueHandle_t command_queue = nullptr;
    QueueHandle_t telemetry_queue = nullptr;
};

// Both backends are always available in the firmware. setup() chooses one at runtime from
// robot_config::communication_type.
void microRosCommunicationTask(void* argument);
void uartCommunicationTask(void* argument);
