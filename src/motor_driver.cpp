#include "motor_driver.hpp"
#include "robot_config.hpp"
#include <cmath>
#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif
bool MotorDriver::begin(){pinMode(robot_config::kMotorRightForwardPin,OUTPUT);pinMode(robot_config::kMotorRightBackwardPin,OUTPUT);pinMode(robot_config::kMotorLeftForwardPin,OUTPUT);pinMode(robot_config::kMotorLeftBackwardPin,OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
 if(!ledcAttachChannel(robot_config::kMotorRightEnablePin,robot_config::kMotorPwmFrequencyHz,robot_config::kMotorPwmResolutionBits,robot_config::kMotorRightPwmChannel))return false;if(!ledcAttachChannel(robot_config::kMotorLeftEnablePin,robot_config::kMotorPwmFrequencyHz,robot_config::kMotorPwmResolutionBits,robot_config::kMotorLeftPwmChannel))return false;
#else
 ledcSetup(robot_config::kMotorRightPwmChannel,robot_config::kMotorPwmFrequencyHz,robot_config::kMotorPwmResolutionBits);ledcSetup(robot_config::kMotorLeftPwmChannel,robot_config::kMotorPwmFrequencyHz,robot_config::kMotorPwmResolutionBits);ledcAttachPin(robot_config::kMotorRightEnablePin,robot_config::kMotorRightPwmChannel);ledcAttachPin(robot_config::kMotorLeftEnablePin,robot_config::kMotorLeftPwmChannel);
#endif
 stop();return true;}
void MotorDriver::duty(int en,int ch,uint32_t v){
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
 (void)ch;ledcWrite(en,v);
#else
 (void)en;ledcWrite(ch,v);
#endif
}
void MotorDriver::one(int f,int b,int en,int ch,float p){p=std::fmax(-(float)robot_config::kPwmMax,std::fmin((float)robot_config::kPwmMax,p));if(std::fabs(p)<robot_config::kPwmDeadzone)p=0;digitalWrite(f,p>0?HIGH:LOW);digitalWrite(b,p<0?HIGH:LOW);duty(en,ch,(uint32_t)std::lround(std::fabs(p)));}void MotorDriver::write(float r,float l){one(robot_config::kMotorRightForwardPin,robot_config::kMotorRightBackwardPin,robot_config::kMotorRightEnablePin,robot_config::kMotorRightPwmChannel,r);one(robot_config::kMotorLeftForwardPin,robot_config::kMotorLeftBackwardPin,robot_config::kMotorLeftEnablePin,robot_config::kMotorLeftPwmChannel,l);}void MotorDriver::stop(){write(0,0);}
