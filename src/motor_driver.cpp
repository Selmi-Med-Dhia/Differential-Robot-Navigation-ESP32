#include "motor_driver.hpp"

#include <cmath>

#include "robot_config.hpp"

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif

bool MotorDriver::begin() {
    pinMode(robot_config::motor_R_forward_pin, OUTPUT);
    pinMode(robot_config::motor_R_backward_pin, OUTPUT);
    pinMode(robot_config::motor_L_forward_pin, OUTPUT);
    pinMode(robot_config::motor_L_backward_pin, OUTPUT);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttachChannel(robot_config::motor_R_enable_pin,
                           robot_config::motor_pwm_frequency_hz,
                           robot_config::motor_pwm_resolution_bits,
                           robot_config::motor_R_pwm_channel)) {
        return false;
    }

    if (!ledcAttachChannel(robot_config::motor_L_enable_pin,
                           robot_config::motor_pwm_frequency_hz,
                           robot_config::motor_pwm_resolution_bits,
                           robot_config::motor_L_pwm_channel)) {
        return false;
    }
#else
    ledcSetup(robot_config::motor_R_pwm_channel,
              robot_config::motor_pwm_frequency_hz,
              robot_config::motor_pwm_resolution_bits);
    ledcSetup(robot_config::motor_L_pwm_channel,
              robot_config::motor_pwm_frequency_hz,
              robot_config::motor_pwm_resolution_bits);

    ledcAttachPin(robot_config::motor_R_enable_pin, robot_config::motor_R_pwm_channel);
    ledcAttachPin(robot_config::motor_L_enable_pin, robot_config::motor_L_pwm_channel);
#endif

    stop();
    return true;
}

void MotorDriver::writeDuty(int enable_pin, int pwm_channel, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)pwm_channel;
    ledcWrite(enable_pin, duty);
#else
    (void)enable_pin;
    ledcWrite(pwm_channel, duty);
#endif
}

void MotorDriver::writeMotor(int forward_pin,
                             int backward_pin,
                             int enable_pin,
                             int pwm_channel,
                             float pwm) {
    // Clamp first so a control bug can never request a duty value outside the configured
    // LEDC range.
    pwm = std::fmax(-static_cast<float>(robot_config::pwm_max),
                    std::fmin(static_cast<float>(robot_config::pwm_max), pwm));

    // PWM below the known motor dead-zone is intentionally converted to zero. The speed
    // controller's static feed-forward is responsible for jumping over this dead-zone.
    if (std::fabs(pwm) < robot_config::pwm_deadzone) {
        pwm = 0.0f;
    }

    digitalWrite(forward_pin, pwm > 0.0f ? HIGH : LOW);
    digitalWrite(backward_pin, pwm < 0.0f ? HIGH : LOW);
    writeDuty(enable_pin,
              pwm_channel,
              static_cast<uint32_t>(std::lround(std::fabs(pwm))));
}

void MotorDriver::write(float pwm_R, float pwm_L) {
    writeMotor(robot_config::motor_R_forward_pin,
               robot_config::motor_R_backward_pin,
               robot_config::motor_R_enable_pin,
               robot_config::motor_R_pwm_channel,
               pwm_R);

    writeMotor(robot_config::motor_L_forward_pin,
               robot_config::motor_L_backward_pin,
               robot_config::motor_L_enable_pin,
               robot_config::motor_L_pwm_channel,
               pwm_L);
}

void MotorDriver::stop() {
    write(0.0f, 0.0f);
}
