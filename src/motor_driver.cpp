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
                             float pwm,
                             int PWM_forward_min,
                             int PWM_backward_min) {
    // Same directional minimum-PWM behavior as run_right_motor()/run_left_motor()
    // in the supplied STM32 and PAMI code.
    pwm = std::fmax(-static_cast<float>(robot_config::PWM_max),
                    std::fmin(static_cast<float>(robot_config::PWM_max), pwm));

    if (pwm > 0.0f && pwm < static_cast<float>(PWM_forward_min)) {
        pwm = 0.0f;
    } else if (pwm < 0.0f && pwm > static_cast<float>(PWM_backward_min)) {
        pwm = 0.0f;
    }

    digitalWrite(forward_pin, pwm > 0.0f ? HIGH : LOW);
    digitalWrite(backward_pin, pwm < 0.0f ? HIGH : LOW);

    // STM32/PAMI PWM variables are integer-valued; truncate the magnitude here
    // instead of introducing a different rounding rule.
    writeDuty(enable_pin,
              pwm_channel,
              static_cast<uint32_t>(std::fabs(pwm)));
}

void MotorDriver::write(float pwm_R, float pwm_L) {
    writeMotor(robot_config::motor_R_forward_pin,
               robot_config::motor_R_backward_pin,
               robot_config::motor_R_enable_pin,
               robot_config::motor_R_pwm_channel,
               pwm_R,
               robot_config::PWM_R_forward_min,
               robot_config::PWM_R_backward_min);

    writeMotor(robot_config::motor_L_forward_pin,
               robot_config::motor_L_backward_pin,
               robot_config::motor_L_enable_pin,
               robot_config::motor_L_pwm_channel,
               pwm_L,
               robot_config::PWM_L_forward_min,
               robot_config::PWM_L_backward_min);
}

void MotorDriver::stop() {
    write(0.0f, 0.0f);
}
