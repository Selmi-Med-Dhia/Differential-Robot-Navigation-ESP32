#pragma once

#include <Arduino.h>

class MotorDriver {
public:
    bool begin();
    void write(float right_pwm, float left_pwm);
    void stop();

private:
    void writeMotor(int forward_pin,
                    int backward_pin,
                    int enable_pin,
                    int pwm_channel,
                    float pwm);
    void writeDuty(int enable_pin, int pwm_channel, uint32_t duty);
};
