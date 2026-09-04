#pragma once
#include <Arduino.h>
class MotorDriver{public:bool begin();void write(float r,float l);void stop();private:void one(int f,int b,int en,int ch,float pwm);void duty(int en,int ch,uint32_t v);};
