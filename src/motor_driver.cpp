#include "motor_driver.hpp"

#include <cmath>

#include "robot_config.hpp"

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif

bool MotorDriver::begin() {
    pinMode(robot_config::kMotorRightForwardPin, OUTPUT);
    pinMode(robot_config::kMotorRightBackwardPin, OUTPUT);
    pinMode(robot_config::kMotorLeftForwardPin, OUTPUT);
    pinMode(robot_config::kMotorLeftBackwardPin, OUTPUT);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttachChannel(robot_config::kMotorRightEnablePin,
                           robot_config::kMotorPwmFrequencyHz,
                           robot_config::kMotorPwmResolutionBits,
                           robot_config::kMotorRightPwmChannel)) {
        return false;
    }

    if (!ledcAttachChannel(robot_config::kMotorLeftEnablePin,
                           robot_config::kMotorPwmFrequencyHz,
                           robot_config::kMotorPwmResolutionBits,
                           robot_config::kMotorLeftPwmChannel)) {
        return false;
    }
#else
    ledcSetup(robot_config::kMotorRightPwmChannel,
              robot_config::kMotorPwmFrequencyHz,
              robot_config::kMotorPwmResolutionBits);
    ledcSetup(robot_config::kMotorLeftPwmChannel,
              robot_config::kMotorPwmFrequencyHz,
              robot_config::kMotorPwmResolutionBits);

    ledcAttachPin(robot_config::kMotorRightEnablePin,
                  robot_config::kMotorRightPwmChannel);
    ledcAttachPin(robot_config::kMotorLeftEnablePin,
                  robot_config::kMotorLeftPwmChannel);
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
    pwm = std::fmax(-static_cast<float>(robot_config::kPwmMax),
                    std::fmin(static_cast<float>(robot_config::kPwmMax), pwm));

    if (std::fabs(pwm) < robot_config::kPwmDeadzone) {
        pwm = 0.0f;
    }

    digitalWrite(forward_pin, pwm > 0.0f ? HIGH : LOW);
    digitalWrite(backward_pin, pwm < 0.0f ? HIGH : LOW);

    writeDuty(enable_pin,
              pwm_channel,
              static_cast<uint32_t>(std::lround(std::fabs(pwm))));
}

void MotorDriver::write(float right_pwm, float left_pwm) {
    writeMotor(robot_config::kMotorRightForwardPin,
               robot_config::kMotorRightBackwardPin,
               robot_config::kMotorRightEnablePin,
               robot_config::kMotorRightPwmChannel,
               right_pwm);

    writeMotor(robot_config::kMotorLeftForwardPin,
               robot_config::kMotorLeftBackwardPin,
               robot_config::kMotorLeftEnablePin,
               robot_config::kMotorLeftPwmChannel,
               left_pwm);
}

void MotorDriver::stop() {
    write(0.0f, 0.0f);
}
