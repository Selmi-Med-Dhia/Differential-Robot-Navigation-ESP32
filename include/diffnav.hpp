#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace diffnav {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

inline float clamp(float value, float minimum, float maximum) {
    return std::max(minimum, std::min(value, maximum));
}

inline float wrapAngle(float angle) {
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

inline float sign(float value) {
    if (value > 0.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
}

inline float distance2D(float x, float y) {
    return std::sqrt(x * x + y * y);
}

struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
};

struct Pose2D {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float theta_rad = 0.0f;
    float theta_unwrapped_rad = 0.0f;
};

struct WheelSpeeds {
    float right_mm_s = 0.0f;
    float left_mm_s = 0.0f;
};

struct BodyVelocity {
    float linear_mm_s = 0.0f;
    float angular_rad_s = 0.0f;
};

struct OdometryState {
    Pose2D pose{};
    WheelSpeeds wheel_speed{};
    BodyVelocity body_velocity{};
    int64_t right_ticks = 0;
    int64_t left_ticks = 0;
};

enum class MotionMode : uint8_t {
    IDLE,
    VELOCITY,
    ALIGN,
    DRIVE_LINE,
    ROTATE,
    FINAL_ORIENT,
    EMERGENCY_STOP,
    FAULT
};

enum class MotionResult : uint8_t {
    IDLE,
    RUNNING,
    SUCCEEDED,
    CANCELLED,
    FAULTED
};

enum class FaultCode : uint8_t {
    NONE,
    ENCODER_INIT,
    CONTROL_TIMING,
    STALL_RIGHT,
    STALL_LEFT,
    INVALID_COMMAND
};

struct MotionStatus {
    MotionMode mode = MotionMode::IDLE;
    MotionResult result = MotionResult::IDLE;
    FaultCode fault = FaultCode::NONE;
    float progress_mm = 0.0f;
    float remaining_mm = 0.0f;
    float cross_track_mm = 0.0f;
    float heading_error_rad = 0.0f;
};

enum class NavCommandType : uint8_t {
    NONE,
    STOP,
    EMERGENCY_STOP,
    CLEAR_EMERGENCY,
    VELOCITY,
    GO_TO,
    MOVE_DISTANCE,
    ROTATE_RELATIVE,
    ORIENT_ABSOLUTE,
    SET_POSE
};

struct NavCommand {
    NavCommandType type = NavCommandType::NONE;
    float p0 = 0.0f;
    float p1 = 0.0f;
    float p2 = 0.0f;
    float p3 = 0.0f;
    int8_t direction = 1;
    bool use_final_heading = false;
};

struct OdometryConfig {
    float right_wheel_diameter_mm = 34.73184056f;
    float left_wheel_diameter_mm = 34.69274232f;
    float track_width_mm = 110.37343379f;
    int32_t counts_per_revolution = 2800;
    float speed_filter_tau_s = 0.030f;
};

struct NavigatorConfig {
    float track_width_mm = 110.37343379f;
    float max_wheel_speed_mm_s = 750.0f;

    float line_accel_mm_s2 = 900.0f;
    float line_decel_mm_s2 = 1200.0f;
    float line_min_speed_mm_s = 55.0f;
    float default_line_speed_mm_s = 450.0f;

    float rotate_wheel_speed_mm_s = 260.0f;
    float rotate_accel_mm_s2 = 700.0f;
    float rotate_decel_mm_s2 = 900.0f;
    float rotate_min_wheel_speed_mm_s = 35.0f;

    float heading_kp = 4.5f;
    float cross_track_gain_s_inv = 3.0f;
    float stanley_gain = 2.4f;
    float stanley_softening_mm_s = 90.0f;
    float max_angular_rate_rad_s = 5.5f;

    float final_position_kp_s_inv = 3.0f;
    float final_speed_limit_mm_s = 120.0f;
    float final_heading_kp = 5.0f;

    float position_tolerance_mm = 2.0f;
    float cross_track_tolerance_mm = 3.0f;
    float heading_tolerance_rad = 0.0174532925f;
    float align_tolerance_rad = 0.0349065850f;
    float stopped_speed_tolerance_mm_s = 12.0f;
    uint16_t settle_cycles = 8;
};

struct WheelControllerConfig {
    float kp = 0.80f;
    float ki = 3.0f;
    float kd = 0.0f;
    float derivative_tau_s = 0.025f;
    float antiwindup_gain_s_inv = 8.0f;
    float integral_limit_pwm = 260.0f;
    float pwm_limit = 1023.0f;
    float static_feedforward_pwm = 90.0f;
    float velocity_feedforward_pwm_per_mm_s = 1.20f;
    float pwm_slew_per_s = 9000.0f;
};

WheelSpeeds bodyToWheels(float linear_mm_s, float angular_rad_s, float track_width_mm);
WheelSpeeds limitWheelSpeeds(WheelSpeeds target, float max_wheel_speed_mm_s);
float profileSpeed(float progress_mm,
                   float remaining_mm,
                   float cruise_speed_mm_s,
                   float acceleration_mm_s2,
                   float deceleration_mm_s2,
                   float minimum_speed_mm_s);
int32_t extendLegacyPcntDelta(int16_t previous, int16_t current, int32_t limit = 30000);

class WheelSpeedController {
public:
    explicit WheelSpeedController(WheelControllerConfig config = {});

    void reset(float output_pwm = 0.0f);
    float update(float target_mm_s, float measured_mm_s, float dt_s);

private:
    WheelControllerConfig config_{};
    float integral_pwm_ = 0.0f;
    float derivative_mm_s2_ = 0.0f;
    float previous_measurement_mm_s_ = 0.0f;
    float previous_output_pwm_ = 0.0f;
    bool initialized_ = false;
};

class DifferentialOdometry {
public:
    explicit DifferentialOdometry(OdometryConfig config = {});

    void reset(const Pose2D& pose, int64_t right_ticks, int64_t left_ticks);
    void setPose(const Pose2D& pose);
    void update(int64_t right_ticks, int64_t left_ticks, float dt_s);

    const OdometryState& state() const { return state_; }

    float rightTicksToDistance(int64_t ticks) const;
    float leftTicksToDistance(int64_t ticks) const;
    int64_t rightDistanceToTicks(float distance_mm) const;
    int64_t leftDistanceToTicks(float distance_mm) const;

private:
    OdometryConfig config_{};
    OdometryState state_{};
    int64_t previous_right_ticks_ = 0;
    int64_t previous_left_ticks_ = 0;
    bool initialized_ = false;
};

class Navigator {
public:
    explicit Navigator(NavigatorConfig config = {});

    void stop();
    void emergencyStop(FaultCode fault = FaultCode::NONE);
    void clearEmergency();

    void commandVelocity(float linear_mm_s, float angular_rad_s);
    void commandMoveDistance(const Pose2D& current_pose, float distance_mm, float speed_mm_s);
    void commandRotateRelative(const Pose2D& current_pose, float angle_rad, float wheel_speed_mm_s = 0.0f);
    void commandOrientAbsolute(const Pose2D& current_pose, float angle_rad, float wheel_speed_mm_s = 0.0f);
    void commandGoTo(const Pose2D& current_pose,
                     float target_x_mm,
                     float target_y_mm,
                     float speed_mm_s,
                     int direction,
                     bool use_final_heading = false,
                     float final_heading_rad = 0.0f);

    WheelSpeeds update(const OdometryState& odometry, float dt_s);

    const MotionStatus& status() const { return status_; }
    MotionMode mode() const { return status_.mode; }
    bool busy() const { return status_.result == MotionResult::RUNNING; }

private:
    void beginLine(const Pose2D& current_pose, Point2D target, float speed_mm_s, int direction);
    WheelSpeeds updateLine(const OdometryState& odometry);
    WheelSpeeds updateRotation(const OdometryState& odometry);
    WheelSpeeds updateAlignment(const OdometryState& odometry);
    WheelSpeeds bodyCommand(float linear_mm_s, float angular_rad_s) const;
    bool robotStopped(const OdometryState& odometry) const;
    void markSucceeded();

    NavigatorConfig config_{};
    MotionStatus status_{};

    float velocity_linear_mm_s_ = 0.0f;
    float velocity_angular_rad_s_ = 0.0f;

    Point2D line_start_{};
    Point2D line_target_{};
    float line_unit_x_ = 1.0f;
    float line_unit_y_ = 0.0f;
    float line_length_mm_ = 0.0f;
    float line_heading_rad_ = 0.0f;
    float line_cruise_speed_mm_s_ = 0.0f;
    int line_direction_ = 1;

    float target_heading_unwrapped_rad_ = 0.0f;
    float rotation_length_mm_ = 0.0f;
    float rotation_wheel_speed_mm_s_ = 0.0f;

    Point2D goto_target_{};
    float goto_speed_mm_s_ = 0.0f;
    int goto_direction_ = 1;
    bool goto_active_ = false;
    bool goto_use_final_heading_ = false;
    float goto_final_heading_rad_ = 0.0f;

    uint16_t settle_counter_ = 0;
};

}  // namespace diffnav
