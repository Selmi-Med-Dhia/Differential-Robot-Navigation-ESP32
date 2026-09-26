#pragma once

#include <cstdint>

#include "diffnav.hpp"

namespace robot_config {

// -----------------------------------------------------------------------------
// Communication selection
// -----------------------------------------------------------------------------
// Change only this line when you want to switch communication method.
// No PlatformIO environment change is needed.
enum class CommunicationType : uint8_t {
    MICRO_ROS,
    UART,
};

constexpr CommunicationType communication_type = CommunicationType::MICRO_ROS;

// -----------------------------------------------------------------------------
// Motor pins and PWM
// -----------------------------------------------------------------------------
constexpr int motor_R_forward_pin = 19;
constexpr int motor_R_backward_pin = 18;
constexpr int motor_L_forward_pin = 21;
constexpr int motor_L_backward_pin = 22;
constexpr int motor_R_enable_pin = 17;
constexpr int motor_L_enable_pin = 23;

constexpr int motor_R_pwm_channel = 8;
constexpr int motor_L_pwm_channel = 9;
constexpr uint32_t motor_pwm_frequency_hz = 20000;
constexpr uint8_t motor_pwm_resolution_bits = 10;

// Keep the ESP32/PAMI PWM range and calibrated motor-start thresholds.
// The PID formula below is the STM32 formula, but these hardware limits belong
// to this ESP32 motor driver and must stay in its native 10-bit PWM scale.
constexpr int PWM_max = 1023;
constexpr int PWM_R_forward_min = 90;
constexpr int PWM_L_forward_min = 90;
constexpr int PWM_R_backward_min = -90;
constexpr int PWM_L_backward_min = -90;

// Same offset distance used by the STM32 set_target_speeds() implementation.
constexpr int PWM_offset_margin = 30;

// STM32 speed PID gains.
constexpr float kpS_R = 15.0f;
constexpr float kdS_R = 0.0f;
constexpr float kiS_R = 200.0f;
constexpr float kpS_L = 15.0f;
constexpr float kdS_L = 0.0f;
constexpr float kiS_L = 200.0f;

// -----------------------------------------------------------------------------
// Encoder pins
// -----------------------------------------------------------------------------
// PCNT decodes both A and B phases, therefore the configured CPR is x4.
constexpr int encoder_R_A_pin = 32;
constexpr int encoder_R_B_pin = 15;
constexpr int encoder_L_A_pin = 16;
constexpr int encoder_L_B_pin = 4;

// Keep both counts positive when the robot moves forward. If one wheel counts backwards,
// change only the corresponding direction from +1 to -1.
constexpr int encoder_direction_R = 1;
constexpr int encoder_direction_L = 1;
constexpr int32_t encoder_counts_per_revolution = 2800;
constexpr uint32_t encoder_glitch_filter_ns = 1000;

// -----------------------------------------------------------------------------
// Mechanical calibration
// -----------------------------------------------------------------------------
// The naming intentionally follows the original PAMI/STM32 navigation code: wheel
// diameters plus wheel spacing, rather than generic robotics names such as track width.
constexpr float wheel_diameter_R_mm = 34.731840562040404f;
constexpr float wheel_diameter_L_mm = 34.6927423194805f;
constexpr float wheel_spacing_mm = 110.37343379225145f;

// -----------------------------------------------------------------------------
// Real-time control
// -----------------------------------------------------------------------------
// TIM7 in the STM32 project runs the speed loop at 1 kHz. Keep the ESP32
// controller at the same update rate because the STM derivative/PWM filters
// use fixed per-sample coefficients.
constexpr uint32_t control_period_us = 1000;              // 1 kHz
constexpr uint32_t control_timing_fault_threshold_us = 30000;
constexpr uint8_t control_timing_fault_cycles = 5;
constexpr uint32_t telemetry_period_ms = 40;

// -----------------------------------------------------------------------------
// Stall detection
// -----------------------------------------------------------------------------
constexpr float stall_target_speed_threshold_mm_s = 140.0f;
constexpr float stall_measured_speed_threshold_mm_s = 18.0f;
constexpr float stall_pwm_threshold = 330.0f;
constexpr uint32_t stall_timeout_ms = 350;

// -----------------------------------------------------------------------------
// micro-ROS
// -----------------------------------------------------------------------------
constexpr uint32_t ros_spin_period_ms = 5;
constexpr uint32_t ros_agent_ping_period_ms = 1000;

// -----------------------------------------------------------------------------
// UART
// -----------------------------------------------------------------------------
// UART uses Serial2 so USB Serial remains available and micro-ROS can still use Serial.
constexpr int uart_rx_pin = 25;
constexpr int uart_tx_pin = 26;
constexpr uint32_t uart_baud_rate = 115200;
constexpr uint32_t uart_telemetry_period_ms = 100;

inline diffnav::OdometryConfig odometryConfig() {
    diffnav::OdometryConfig config;
    config.wheel_diameter_R_mm = wheel_diameter_R_mm;
    config.wheel_diameter_L_mm = wheel_diameter_L_mm;
    config.wheel_spacing_mm = wheel_spacing_mm;
    config.encoder_counts_per_revolution = encoder_counts_per_revolution;
    return config;
}

inline diffnav::NavigatorConfig navigatorConfig() {
    diffnav::NavigatorConfig config;
    config.wheel_spacing_mm = wheel_spacing_mm;
    return config;
}

inline diffnav::WheelControllerConfig wheelControllerConfig_R() {
    diffnav::WheelControllerConfig config;
    config.speed_kp = kpS_R;
    config.speed_kd = kdS_R;
    config.speed_ki = kiS_R;
    config.pwm_limit = PWM_max;
    config.pwm_forward_min = PWM_R_forward_min;
    config.pwm_backward_min = PWM_R_backward_min;
    config.pwm_offset_margin = PWM_offset_margin;
    return config;
}

inline diffnav::WheelControllerConfig wheelControllerConfig_L() {
    diffnav::WheelControllerConfig config;
    config.speed_kp = kpS_L;
    config.speed_kd = kdS_L;
    config.speed_ki = kiS_L;
    config.pwm_limit = PWM_max;
    config.pwm_forward_min = PWM_L_forward_min;
    config.pwm_backward_min = PWM_L_backward_min;
    config.pwm_offset_margin = PWM_offset_margin;
    return config;
}

}  // namespace robot_config
