#pragma once
#include "diffnav.hpp"
namespace robot_config {
constexpr int kMotorRightForwardPin=19,kMotorRightBackwardPin=18,kMotorLeftForwardPin=21,kMotorLeftBackwardPin=22,kMotorRightEnablePin=17,kMotorLeftEnablePin=23;
constexpr int kMotorRightPwmChannel=8,kMotorLeftPwmChannel=9;constexpr uint32_t kMotorPwmFrequencyHz=20000;constexpr uint8_t kMotorPwmResolutionBits=10;constexpr int kPwmMax=1023,kPwmDeadzone=90;
constexpr int kRightEncoderAPin=32,kRightEncoderBPin=15,kLeftEncoderAPin=16,kLeftEncoderBPin=4,kRightEncoderSign=1,kLeftEncoderSign=1;constexpr int32_t kCountsPerRevolutionX4=2800;constexpr uint32_t kEncoderGlitchFilterNs=1000;
constexpr float kRightWheelDiameterMm=34.731840562040404f,kLeftWheelDiameterMm=34.6927423194805f,kTrackWidthMm=110.37343379225145f;
constexpr uint32_t kControlPeriodUs=5000,kTelemetryPeriodMs=40,kRosSpinPeriodMs=5;
constexpr float kStallTargetThresholdMmS=140,kStallMeasuredThresholdMmS=18,kStallPwmThreshold=330;constexpr uint32_t kStallTimeoutMs=350;
inline diffnav::OdometryConfig odometryConfig(){diffnav::OdometryConfig c;c.right_wheel_diameter_mm=kRightWheelDiameterMm;c.left_wheel_diameter_mm=kLeftWheelDiameterMm;c.track_width_mm=kTrackWidthMm;c.counts_per_revolution=kCountsPerRevolutionX4;return c;}
inline diffnav::NavigatorConfig navigatorConfig(){diffnav::NavigatorConfig c;c.track_width_mm=kTrackWidthMm;return c;}
inline diffnav::WheelControllerConfig wheelControllerConfig(){diffnav::WheelControllerConfig c;c.pwm_limit=kPwmMax;c.static_feedforward_pwm=kPwmDeadzone;return c;}
}
