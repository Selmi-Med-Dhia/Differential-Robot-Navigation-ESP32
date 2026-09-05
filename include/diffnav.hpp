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
    return value > 0.0f ? 1.0f : (value < 0.0f ? -1.0f : 0.0f);
}

inline float distance2D(float x, float y) {
    return std::sqrt(x * x + y * y);
}

struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
};

// phi is the wrapped robot orientation in [-pi, pi].
// absolute_phi keeps accumulating through complete turns and is useful for precise rotations.
struct Pose2D {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float phi_rad = 0.0f;
    float absolute_phi_rad = 0.0f;
};

struct WheelSpeeds {
    float speed_R_mm_s = 0.0f;
    float speed_L_mm_s = 0.0f;
};

struct RobotSpeed {
    float speed_mm_s = 0.0f;
    float angular_speed_rad_s = 0.0f;
};

struct OdometryState {
    Pose2D pose{};
    WheelSpeeds wheel_speed{};
    RobotSpeed robot_speed{};
    int64_t encoder_count_R = 0;
    int64_t encoder_count_L = 0;
};

enum class MotionMode : uint8_t {
    IDLE,
    VELOCITY,
    ALIGN,
    DRIVE_LINE,
    ROTATE,
    FINAL_ORIENT,
    EMERGENCY_STOP,
    FAULT,
};

enum class MotionResult : uint8_t {
    IDLE,
    RUNNING,
    SUCCEEDED,
    CANCELLED,
    FAULTED,
};

enum class FaultCode : uint8_t {
    NONE,
    ENCODER_INIT,
    CONTROL_TIMING,
    STALL_RIGHT,
    STALL_LEFT,
    INVALID_COMMAND,
};

struct MotionStatus {
    MotionMode mode = MotionMode::IDLE;
    MotionResult result = MotionResult::IDLE;
    FaultCode fault = FaultCode::NONE;
    float progress_mm = 0.0f;
    float remaining_mm = 0.0f;
    float lane_error_mm = 0.0f;
    float phi_error_rad = 0.0f;
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
    SET_POSE,
};

// The command structure uses named fields instead of p0/p1/p2. Only the fields relevant
// to the selected command type are used.
struct NavCommand {
    NavCommandType type = NavCommandType::NONE;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float phi_rad = 0.0f;
    float speed_mm_s = 0.0f;
    float distance_mm = 0.0f;
    float angular_speed_rad_s = 0.0f;
    int8_t direction = 1;
    bool use_final_phi = false;
};

struct OdometryConfig {
    float wheel_diameter_R_mm = 34.73184056f;
    float wheel_diameter_L_mm = 34.69274232f;
    float wheel_spacing_mm = 110.37343379f;
    int32_t encoder_counts_per_revolution = 2800;
    float speed_filter_tau_s = 0.030f;
};

// These names intentionally follow the original navigation code terminology.
struct NavigatorConfig {
    float wheel_spacing_mm = 110.37343379f;
    float max_wheel_speed_mm_s = 750.0f;

    float ramping_acceleration_mm_s2 = 900.0f;
    float breaking_acceleration_mm_s2 = 1200.0f;
    float minimum_cruise_speed_mm_s = 55.0f;
    float default_cruise_speed_mm_s = 450.0f;

    float rotating_speed_mm_s = 260.0f;
    float rotating_ramping_acceleration_mm_s2 = 700.0f;
    float rotating_breaking_acceleration_mm_s2 = 900.0f;
    float rotating_minimum_speed_mm_s = 35.0f;

    float phi_correction_kp = 4.5f;
    float lane_correction_gain_s_inv = 3.0f;
    float lane_correction_angle_gain = 2.4f;
    float lane_correction_softening_mm_s = 90.0f;
    float maximum_angular_speed_rad_s = 5.5f;

    float final_position_kp_s_inv = 3.0f;
    float final_speed_limit_mm_s = 120.0f;
    float final_phi_kp = 5.0f;

    float position_tolerance_mm = 2.0f;
    float lane_tolerance_mm = 3.0f;
    float phi_tolerance_rad = 0.0174532925f;
    float orientation_tolerance_rad = 0.0349065850f;
    float stopped_speed_tolerance_mm_s = 12.0f;
    uint16_t settle_cycles = 8;
};

struct WheelControllerConfig {
    float speed_kp = 0.80f;
    float speed_ki = 3.0f;
    float speed_kd = 0.0f;
    float derivative_filter_tau_s = 0.025f;
    float antiwindup_gain_s_inv = 8.0f;
    float speed_integral_limit_pwm = 260.0f;
    float pwm_limit = 1023.0f;
    float static_feedforward_pwm = 90.0f;
    float velocity_feedforward_pwm_per_mm_s = 1.20f;
    float pwm_slew_per_s = 9000.0f;
};

WheelSpeeds robotSpeedToWheelSpeeds(float speed_mm_s,
                                    float angular_speed_rad_s,
                                    float wheel_spacing_mm);
WheelSpeeds limitWheelSpeeds(WheelSpeeds target, float max_wheel_speed_mm_s);
float calculateProfileSpeed(float distance_travelled_mm,
                            float distance_remaining_mm,
                            float cruise_speed_mm_s,
                            float ramping_acceleration_mm_s2,
                            float breaking_acceleration_mm_s2,
                            float minimum_speed_mm_s);
int32_t extendPcntDelta(int16_t previous_count, int16_t current_count, int32_t limit = 30000);

class WheelSpeedController {
public:
    explicit WheelSpeedController(WheelControllerConfig config = {});

    void reset(float pwm = 0.0f);
    float update(float target_speed_mm_s, float measured_speed_mm_s, float dt_s);

private:
    WheelControllerConfig config_{};
    float speed_integral_pwm_ = 0.0f;
    float filtered_speed_derivative_mm_s2_ = 0.0f;
    float previous_speed_mm_s_ = 0.0f;
    float previous_pwm_ = 0.0f;
    bool initialized_ = false;
};

class DifferentialOdometry {
public:
    explicit DifferentialOdometry(OdometryConfig config = {});

    void reset(const Pose2D& pose, int64_t encoder_count_R, int64_t encoder_count_L);
    void setPose(const Pose2D& pose);
    void update(int64_t encoder_count_R, int64_t encoder_count_L, float dt_s);

    const OdometryState& state() const { return state_; }

    float encoderCountToDistance_R(int64_t encoder_count) const;
    float encoderCountToDistance_L(int64_t encoder_count) const;

private:
    OdometryConfig config_{};
    OdometryState state_{};
    int64_t previous_encoder_count_R_ = 0;
    int64_t previous_encoder_count_L_ = 0;
    bool initialized_ = false;
};

class Navigator {
public:
    explicit Navigator(NavigatorConfig config = {});

    void stop();
    void emergencyBreak(FaultCode fault = FaultCode::NONE);
    void clearEmergencyBreak();

    void setSpeed(float speed_mm_s, float angular_speed_rad_s);
    void moveForward(const Pose2D& current_pose, float distance_mm, float cruise_speed_mm_s);
    void rotate(const Pose2D& current_pose, float angle_rad, float rotating_speed_mm_s = 0.0f);
    void orientate(const Pose2D& current_pose, float phi_rad, float rotating_speed_mm_s = 0.0f);
    void goTo(const Pose2D& current_pose,
              float target_x_mm,
              float target_y_mm,
              float cruise_speed_mm_s,
              int direction,
              bool use_final_phi = false,
              float final_phi_rad = 0.0f);

    WheelSpeeds update(const OdometryState& odometry, float dt_s);

    const MotionStatus& status() const { return status_; }
    MotionMode mode() const { return status_.mode; }

private:
    void startStraightMovement(const Pose2D& current_pose,
                               Point2D target_position,
                               float cruise_speed_mm_s,
                               int direction);
    WheelSpeeds updateStraightMovement(const OdometryState& odometry);
    WheelSpeeds updateRotation(const OdometryState& odometry);
    WheelSpeeds updateAlignment(const OdometryState& odometry);
    WheelSpeeds makeWheelSpeedCommand(float speed_mm_s, float angular_speed_rad_s) const;
    bool robotStopped(const OdometryState& odometry) const;
    void movementFinished();

    NavigatorConfig config_{};
    MotionStatus status_{};

    // Used only in direct velocity mode.
    float commanded_speed_mm_s_ = 0.0f;
    float commanded_angular_speed_rad_s_ = 0.0f;

    // Straight movement state.
    Point2D starting_position_{};
    Point2D target_position_{};
    float direction_x_ = 1.0f;
    float direction_y_ = 0.0f;
    float target_distance_mm_ = 0.0f;
    float starting_phi_rad_ = 0.0f;
    float cruise_speed_mm_s_ = 0.0f;
    int direction_ = 1;

    // Rotation/orientation state.
    float target_absolute_phi_rad_ = 0.0f;
    float rotation_distance_mm_ = 0.0f;
    float rotating_speed_mm_s_ = 0.0f;

    // go_to state. The robot first aligns, then uses the same straight movement controller.
    Point2D go_to_target_{};
    float go_to_cruise_speed_mm_s_ = 0.0f;
    int go_to_direction_ = 1;
    bool go_to_active_ = false;
    bool go_to_use_final_phi_ = false;
    float go_to_final_phi_rad_ = 0.0f;

    uint16_t settle_counter_ = 0;
};

}  // namespace diffnav
