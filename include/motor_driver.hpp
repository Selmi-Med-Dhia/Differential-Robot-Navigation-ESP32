#pragma once

#include <Arduino.h>

// Thin hardware layer for the two H-bridge motor channels. Navigation and PID code never
// touches GPIO directly; it only sends signed PWM values to this class.
class MotorDriver {
public:
    bool begin();
    void write(float pwm_R, float pwm_L);
    void stop();

private:
    // Positive PWM drives the forward pin, negative PWM drives the backward pin.
    void writeMotor(int forward_pin,
                    int backward_pin,
                    int enable_pin,
                    int pwm_channel,
                    float pwm);

    // Arduino-ESP32 2.x and 3.x expose slightly different LEDC APIs. The implementation
    // hides that difference from the rest of the project.
    void writeDuty(int enable_pin, int pwm_channel, uint32_t duty);
};
