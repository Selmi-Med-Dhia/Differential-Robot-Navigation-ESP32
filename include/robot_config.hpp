#pragma once

#include "diffnav.hpp"

namespace robot_config {

// Motor driver pins
constexpr int kMotorRightForwardPin = 19;
constexpr int kMotorRightBackwardPin = 18;
constexpr int kMotorLeftForwardPin = 21;
constexpr int kMotorLeftBackwardPin = 22;
constexpr int kMotorRightEnablePin = 17;
constexpr int kMotorLeftEnablePin = 23;

// PWM configuration
constexpr int kMotorRightPwmChannel = 8;
constexpr int kMotorLeftPwmChannel = 9;
constexpr uint32_t kMotorPwmFrequencyHz = 20000;
constexpr uint8_t kMotorPwmResolutionBits = 10;
constexpr int kPwmMax = 1023;
constexpr int kPwmDeadzone = 90;

// Full x4 quadrature encoder configuration
constexpr int kRightEncoderAPin = 32;
constexpr int kRightEncoderBPin = 15;
constexpr int kLeftEncoderAPin = 16;
constexpr int kLeftEncoderBPin = 4;
constexpr int kRightEncoderSign = 1;
constexpr int kLeftEncoderSign = 1;
constexpr int32_t kCountsPerRevolutionX4 = 2800;
constexpr uint32_t kEncoderGlitchFilterNs = 1000;

// Mechanical calibration
constexpr float kRightWheelDiameterMm = 34.731840562040404f;
constexpr float kLeftWheelDiameterMm = 34.6927423194805f;
constexpr float kTrackWidthMm = 110.37343379225145f;

// Real-time control
constexpr uint32_t kControlPeriodUs = 5000;
constexpr uint32_t kControlTimingFaultThresholdUs = 30000;
constexpr uint8_t kControlTimingFaultCycles = 5;
constexpr uint32_t kTelemetryPeriodMs = 40;

// Stall detection
constexpr float kStallTargetThresholdMmS = 140.0f;
constexpr float kStallMeasuredThresholdMmS = 18.0f;
constexpr float kStallPwmThreshold = 330.0f;
constexpr uint32_t kStallTimeoutMs = 350;

// micro-ROS communication
constexpr uint32_t kRosSpinPeriodMs = 5;
constexpr uint32_t kRosAgentPingPeriodMs = 1000;

// Raw UART communication. Change these if GPIO 25/26 are used by your board.
constexpr int kUartRxPin = 25;
constexpr int kUartTxPin = 26;
constexpr uint32_t kUartBaudRate = 115200;
constexpr uint32_t kUartTelemetryPeriodMs = 100;

inline diffnav::OdometryConfig odometryConfig() {
    diffnav::OdometryConfig config;
    config.right_wheel_diameter_mm = kRightWheelDiameterMm;
    config.left_wheel_diameter_mm = kLeftWheelDiameterMm;
    config.track_width_mm = kTrackWidthMm;
    config.counts_per_revolution = kCountsPerRevolutionX4;
    return config;
}

inline diffnav::NavigatorConfig navigatorConfig() {
    diffnav::NavigatorConfig config;
    config.track_width_mm = kTrackWidthMm;
    return config;
}

inline diffnav::WheelControllerConfig wheelControllerConfig() {
    diffnav::WheelControllerConfig config;
    config.pwm_limit = kPwmMax;
    config.static_feedforward_pwm = kPwmDeadzone;
    return config;
}

}  // namespace robot_config
