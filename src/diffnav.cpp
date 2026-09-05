#include "diffnav.hpp"

namespace diffnav {

WheelSpeeds robotSpeedToWheelSpeeds(float speed_mm_s,
                                    float angular_speed_rad_s,
                                    float wheel_spacing_mm) {
    // Differential-drive inverse kinematics:
    // speed_R = robot_speed + omega * wheel_spacing / 2
    // speed_L = robot_speed - omega * wheel_spacing / 2
    const float half_spacing_mm = 0.5f * wheel_spacing_mm;
    return {
        speed_mm_s + angular_speed_rad_s * half_spacing_mm,
        speed_mm_s - angular_speed_rad_s * half_spacing_mm,
    };
}

WheelSpeeds limitWheelSpeeds(WheelSpeeds target, float max_wheel_speed_mm_s) {
    const float largest_speed =
        std::max(std::fabs(target.speed_R_mm_s), std::fabs(target.speed_L_mm_s));

    // Scale both wheels together so curvature is preserved when one side saturates.
    if (largest_speed > max_wheel_speed_mm_s && largest_speed > 1e-6f) {
        const float scale = max_wheel_speed_mm_s / largest_speed;
        target.speed_R_mm_s *= scale;
        target.speed_L_mm_s *= scale;
    }

    return target;
}

float calculateProfileSpeed(float distance_travelled_mm,
                            float distance_remaining_mm,
                            float cruise_speed_mm_s,
                            float ramping_acceleration_mm_s2,
                            float breaking_acceleration_mm_s2,
                            float minimum_speed_mm_s) {
    if (distance_remaining_mm <= 0.0f || cruise_speed_mm_s <= 0.0f) {
        return 0.0f;
    }

    distance_travelled_mm = std::max(0.0f, distance_travelled_mm);
    distance_remaining_mm = std::max(0.0f, distance_remaining_mm);
    ramping_acceleration_mm_s2 = std::max(1.0f, ramping_acceleration_mm_s2);
    breaking_acceleration_mm_s2 = std::max(1.0f, breaking_acceleration_mm_s2);
    minimum_speed_mm_s = std::max(0.0f, minimum_speed_mm_s);

    // Same distance-domain idea as the STM32 navigation code, expressed with the standard
    // kinematic relation v^2 = v0^2 + 2*a*d. The lower of the acceleration and braking
    // limits is used at every point of the trajectory.
    const float ramp_limited_speed = std::sqrt(
        minimum_speed_mm_s * minimum_speed_mm_s +
        2.0f * ramping_acceleration_mm_s2 * distance_travelled_mm);
    const float breaking_limited_speed =
        std::sqrt(2.0f * breaking_acceleration_mm_s2 * distance_remaining_mm);

    float target_speed =
        std::min(cruise_speed_mm_s, std::min(ramp_limited_speed, breaking_limited_speed));

    if (distance_remaining_mm > 1.0f) {
        target_speed = std::max(target_speed, std::min(minimum_speed_mm_s, breaking_limited_speed));
    }

    return std::max(0.0f, target_speed);
}

int32_t extendPcntDelta(int16_t previous_count, int16_t current_count, int32_t limit) {
    // Legacy ESP32 PCNT wraps/reset around the configured limit. Reconstruct the small
    // physical delta so the rest of the code sees a monotonic encoder count.
    int32_t delta = static_cast<int32_t>(current_count) - static_cast<int32_t>(previous_count);
    const int32_t half_limit = limit / 2;

    if (delta < -half_limit) {
        delta += limit;
    } else if (delta > half_limit) {
        delta -= limit;
    }

    return delta;
}

WheelSpeedController::WheelSpeedController(WheelControllerConfig config)
    : config_(config) {}

void WheelSpeedController::reset(float pwm) {
    speed_integral_pwm_ = 0.0f;
    filtered_speed_derivative_mm_s2_ = 0.0f;
    previous_speed_mm_s_ = 0.0f;
    previous_pwm_ = pwm;
    initialized_ = false;
}

float WheelSpeedController::update(float target_speed_mm_s,
                                   float measured_speed_mm_s,
                                   float dt_s) {
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        return previous_pwm_;
    }

    if (std::fabs(target_speed_mm_s) < 0.5f) {
        reset();
        return 0.0f;
    }

    if (!initialized_) {
        previous_speed_mm_s_ = measured_speed_mm_s;
        initialized_ = true;
    }

    const float speed_error = target_speed_mm_s - measured_speed_mm_s;

    // Derivative is taken on the measured speed instead of the error. This avoids a large
    // derivative kick whenever the target speed changes abruptly.
    const float raw_derivative = -(measured_speed_mm_s - previous_speed_mm_s_) / dt_s;
    const float derivative_alpha = dt_s / (config_.derivative_filter_tau_s + dt_s);
    filtered_speed_derivative_mm_s2_ +=
        derivative_alpha * (raw_derivative - filtered_speed_derivative_mm_s2_);
    previous_speed_mm_s_ = measured_speed_mm_s;

    // Feed-forward compensates motor dead-zone and most of the steady-state PWM demand.
    const float feedforward_pwm =
        sign(target_speed_mm_s) *
        (config_.static_feedforward_pwm +
         config_.velocity_feedforward_pwm_per_mm_s * std::fabs(target_speed_mm_s));

    speed_integral_pwm_ += config_.speed_ki * speed_error * dt_s;
    speed_integral_pwm_ = clamp(speed_integral_pwm_,
                                -config_.speed_integral_limit_pwm,
                                config_.speed_integral_limit_pwm);

    const float requested_pwm = feedforward_pwm +
                                config_.speed_kp * speed_error +
                                speed_integral_pwm_ +
                                config_.speed_kd * filtered_speed_derivative_mm_s2_;

    float pwm = clamp(requested_pwm, -config_.pwm_limit, config_.pwm_limit);

    // Back-calculation anti-windup removes integral energy when PWM is saturated.
    speed_integral_pwm_ += config_.antiwindup_gain_s_inv * (pwm - requested_pwm) * dt_s;
    speed_integral_pwm_ = clamp(speed_integral_pwm_,
                                -config_.speed_integral_limit_pwm,
                                config_.speed_integral_limit_pwm);

    // Slew limiting prevents an instantaneous full-power step at the motor driver.
    const float maximum_pwm_change = config_.pwm_slew_per_s * dt_s;
    pwm = clamp(pwm,
                previous_pwm_ - maximum_pwm_change,
                previous_pwm_ + maximum_pwm_change);

    previous_pwm_ = pwm;
    return pwm;
}

DifferentialOdometry::DifferentialOdometry(OdometryConfig config)
    : config_(config) {}

void DifferentialOdometry::reset(const Pose2D& pose,
                                 int64_t encoder_count_R,
                                 int64_t encoder_count_L) {
    state_ = {};
    state_.pose = pose;
    state_.pose.phi_rad = wrapAngle(pose.phi_rad);
    state_.encoder_count_R = encoder_count_R;
    state_.encoder_count_L = encoder_count_L;

    previous_encoder_count_R_ = encoder_count_R;
    previous_encoder_count_L_ = encoder_count_L;
    initialized_ = true;
}

void DifferentialOdometry::setPose(const Pose2D& pose) {
    state_.pose = pose;
    state_.pose.phi_rad = wrapAngle(pose.phi_rad);
}

float DifferentialOdometry::encoderCountToDistance_R(int64_t encoder_count) const {
    return static_cast<float>(encoder_count) * config_.wheel_diameter_R_mm * kPi /
           static_cast<float>(config_.encoder_counts_per_revolution);
}

float DifferentialOdometry::encoderCountToDistance_L(int64_t encoder_count) const {
    return static_cast<float>(encoder_count) * config_.wheel_diameter_L_mm * kPi /
           static_cast<float>(config_.encoder_counts_per_revolution);
}

void DifferentialOdometry::update(int64_t encoder_count_R,
                                  int64_t encoder_count_L,
                                  float dt_s) {
    if (!initialized_) {
        reset(state_.pose, encoder_count_R, encoder_count_L);
        return;
    }

    // A very late sample would produce a misleading wheel-speed estimate. We keep the new
    // encoder baseline but skip integrating that abnormal sample.
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        previous_encoder_count_R_ = encoder_count_R;
        previous_encoder_count_L_ = encoder_count_L;
        state_.encoder_count_R = encoder_count_R;
        state_.encoder_count_L = encoder_count_L;
        return;
    }

    const int64_t encoder_delta_R = encoder_count_R - previous_encoder_count_R_;
    const int64_t encoder_delta_L = encoder_count_L - previous_encoder_count_L_;
    previous_encoder_count_R_ = encoder_count_R;
    previous_encoder_count_L_ = encoder_count_L;

    const float distance_R_mm = encoderCountToDistance_R(encoder_delta_R);
    const float distance_L_mm = encoderCountToDistance_L(encoder_delta_L);
    const float center_distance_mm = 0.5f * (distance_R_mm + distance_L_mm);
    const float delta_phi_rad =
        (distance_R_mm - distance_L_mm) / config_.wheel_spacing_mm;

    // Integrate the differential-drive motion using the exact local circular arc instead
    // of an Euler approximation. For a straight sample delta_phi is practically zero.
    float local_x_mm = 0.0f;
    float local_y_mm = 0.0f;

    if (std::fabs(delta_phi_rad) < 1e-6f) {
        local_x_mm = center_distance_mm;
    } else {
        const float turning_radius_mm = center_distance_mm / delta_phi_rad;
        local_x_mm = turning_radius_mm * std::sin(delta_phi_rad);
        local_y_mm = turning_radius_mm * (1.0f - std::cos(delta_phi_rad));
    }

    const float absolute_phi = state_.pose.absolute_phi_rad;
    const float cos_phi = std::cos(absolute_phi);
    const float sin_phi = std::sin(absolute_phi);

    state_.pose.x_mm += cos_phi * local_x_mm - sin_phi * local_y_mm;
    state_.pose.y_mm += sin_phi * local_x_mm + cos_phi * local_y_mm;
    state_.pose.absolute_phi_rad += delta_phi_rad;
    state_.pose.phi_rad = wrapAngle(state_.pose.absolute_phi_rad);

    // Encoder velocity is low-pass filtered before it reaches the wheel PID controller.
    const float filter_alpha = dt_s / (config_.speed_filter_tau_s + dt_s);
    const float raw_speed_R = distance_R_mm / dt_s;
    const float raw_speed_L = distance_L_mm / dt_s;

    state_.wheel_speed.speed_R_mm_s +=
        filter_alpha * (raw_speed_R - state_.wheel_speed.speed_R_mm_s);
    state_.wheel_speed.speed_L_mm_s +=
        filter_alpha * (raw_speed_L - state_.wheel_speed.speed_L_mm_s);

    state_.robot_speed.speed_mm_s =
        0.5f * (state_.wheel_speed.speed_R_mm_s + state_.wheel_speed.speed_L_mm_s);
    state_.robot_speed.angular_speed_rad_s =
        (state_.wheel_speed.speed_R_mm_s - state_.wheel_speed.speed_L_mm_s) /
        config_.wheel_spacing_mm;

    state_.encoder_count_R = encoder_count_R;
    state_.encoder_count_L = encoder_count_L;
}

Navigator::Navigator(NavigatorConfig config)
    : config_(config) {
    stop();
}

void Navigator::stop() {
    status_ = {};
    status_.mode = MotionMode::IDLE;
    status_.result = MotionResult::IDLE;

    commanded_speed_mm_s_ = 0.0f;
    commanded_angular_speed_rad_s_ = 0.0f;
    go_to_active_ = false;
    settle_counter_ = 0;
}

void Navigator::emergencyBreak(FaultCode fault) {
    status_.mode = fault == FaultCode::NONE ? MotionMode::EMERGENCY_STOP : MotionMode::FAULT;
    status_.result = fault == FaultCode::NONE ? MotionResult::CANCELLED : MotionResult::FAULTED;
    status_.fault = fault;

    commanded_speed_mm_s_ = 0.0f;
    commanded_angular_speed_rad_s_ = 0.0f;
    go_to_active_ = false;
    settle_counter_ = 0;
}

void Navigator::clearEmergencyBreak() {
    stop();
}

void Navigator::setSpeed(float speed_mm_s, float angular_speed_rad_s) {
    if (status_.mode == MotionMode::EMERGENCY_STOP || status_.mode == MotionMode::FAULT) {
        return;
    }

    commanded_speed_mm_s_ = clamp(speed_mm_s,
                                  -config_.max_wheel_speed_mm_s,
                                  config_.max_wheel_speed_mm_s);
    commanded_angular_speed_rad_s_ = clamp(angular_speed_rad_s,
                                           -config_.maximum_angular_speed_rad_s,
                                           config_.maximum_angular_speed_rad_s);

    go_to_active_ = false;
    settle_counter_ = 0;
    status_.mode = MotionMode::VELOCITY;
    status_.result = MotionResult::RUNNING;
    status_.fault = FaultCode::NONE;
}

void Navigator::startStraightMovement(const Pose2D& current_pose,
                                      Point2D target_position,
                                      float cruise_speed_mm_s,
                                      int direction) {
    starting_position_ = {current_pose.x_mm, current_pose.y_mm};
    target_position_ = target_position;

    const float delta_x = target_position_.x - starting_position_.x;
    const float delta_y = target_position_.y - starting_position_.y;
    target_distance_mm_ = distance2D(delta_x, delta_y);

    if (target_distance_mm_ < 1e-3f) {
        movementFinished();
        return;
    }

    direction_x_ = delta_x / target_distance_mm_;
    direction_y_ = delta_y / target_distance_mm_;
    starting_phi_rad_ = std::atan2(delta_y, delta_x);
    direction_ = direction >= 0 ? 1 : -1;

    const float requested_speed =
        cruise_speed_mm_s > 0.0f ? cruise_speed_mm_s : config_.default_cruise_speed_mm_s;
    cruise_speed_mm_s_ = clamp(requested_speed,
                               config_.minimum_cruise_speed_mm_s,
                               config_.max_wheel_speed_mm_s);

    settle_counter_ = 0;
    status_.mode = MotionMode::DRIVE_LINE;
    status_.result = MotionResult::RUNNING;
    status_.fault = FaultCode::NONE;
}

void Navigator::moveForward(const Pose2D& current_pose,
                            float distance_mm,
                            float cruise_speed_mm_s) {
    Point2D target_position{
        current_pose.x_mm + distance_mm * std::cos(current_pose.phi_rad),
        current_pose.y_mm + distance_mm * std::sin(current_pose.phi_rad),
    };

    go_to_active_ = false;
    startStraightMovement(current_pose,
                          target_position,
                          cruise_speed_mm_s,
                          distance_mm >= 0.0f ? 1 : -1);
}

void Navigator::rotate(const Pose2D& current_pose,
                       float angle_rad,
                       float rotating_speed_mm_s) {
    go_to_active_ = false;
    target_absolute_phi_rad_ = current_pose.absolute_phi_rad + angle_rad;

    // During an in-place rotation each wheel travels angle * wheel_spacing / 2.
    rotation_distance_mm_ = std::fabs(angle_rad) * config_.wheel_spacing_mm * 0.5f;
    rotating_speed_mm_s_ = rotating_speed_mm_s > 0.0f
                               ? std::min(rotating_speed_mm_s, config_.max_wheel_speed_mm_s)
                               : config_.rotating_speed_mm_s;

    settle_counter_ = 0;
    status_.mode = MotionMode::ROTATE;
    status_.result = MotionResult::RUNNING;
    status_.fault = FaultCode::NONE;
}

void Navigator::orientate(const Pose2D& current_pose,
                          float phi_rad,
                          float rotating_speed_mm_s) {
    // Use the shortest signed rotation to the requested wrapped orientation.
    const float shortest_angle = wrapAngle(phi_rad - current_pose.phi_rad);
    rotate(current_pose, shortest_angle, rotating_speed_mm_s);
}

void Navigator::goTo(const Pose2D& current_pose,
                     float target_x_mm,
                     float target_y_mm,
                     float cruise_speed_mm_s,
                     int direction,
                     bool use_final_phi,
                     float final_phi_rad) {
    const float delta_x = target_x_mm - current_pose.x_mm;
    const float delta_y = target_y_mm - current_pose.y_mm;

    if (distance2D(delta_x, delta_y) <= config_.position_tolerance_mm) {
        if (use_final_phi) {
            orientate(current_pose, final_phi_rad);
        } else {
            movementFinished();
        }
        return;
    }

    go_to_active_ = true;
    go_to_target_ = {target_x_mm, target_y_mm};
    go_to_cruise_speed_mm_s_ =
        cruise_speed_mm_s > 0.0f ? cruise_speed_mm_s : config_.default_cruise_speed_mm_s;
    go_to_direction_ = direction >= 0 ? 1 : -1;
    go_to_use_final_phi_ = use_final_phi;
    go_to_final_phi_rad_ = wrapAngle(final_phi_rad);

    // First orient the robot towards the line joining its current position to the target.
    // For backwards go_to, the robot points pi radians away from the path direction.
    const float path_phi = std::atan2(delta_y, delta_x);
    const float desired_phi = wrapAngle(path_phi + (go_to_direction_ < 0 ? kPi : 0.0f));
    const float phi_delta = wrapAngle(desired_phi - current_pose.phi_rad);

    target_absolute_phi_rad_ = current_pose.absolute_phi_rad + phi_delta;
    rotation_distance_mm_ = std::fabs(phi_delta) * config_.wheel_spacing_mm * 0.5f;
    rotating_speed_mm_s_ = config_.rotating_speed_mm_s;

    settle_counter_ = 0;
    status_.mode = MotionMode::ALIGN;
    status_.result = MotionResult::RUNNING;
    status_.fault = FaultCode::NONE;
}

WheelSpeeds Navigator::makeWheelSpeedCommand(float speed_mm_s,
                                             float angular_speed_rad_s) const {
    const float limited_angular_speed = clamp(angular_speed_rad_s,
                                              -config_.maximum_angular_speed_rad_s,
                                              config_.maximum_angular_speed_rad_s);

    return limitWheelSpeeds(
        robotSpeedToWheelSpeeds(speed_mm_s,
                                limited_angular_speed,
                                config_.wheel_spacing_mm),
        config_.max_wheel_speed_mm_s);
}

bool Navigator::robotStopped(const OdometryState& odometry) const {
    return std::fabs(odometry.wheel_speed.speed_R_mm_s) < config_.stopped_speed_tolerance_mm_s &&
           std::fabs(odometry.wheel_speed.speed_L_mm_s) < config_.stopped_speed_tolerance_mm_s;
}

void Navigator::movementFinished() {
    status_.mode = MotionMode::IDLE;
    status_.result = MotionResult::SUCCEEDED;
    status_.fault = FaultCode::NONE;
    go_to_active_ = false;
    settle_counter_ = 0;
}

WheelSpeeds Navigator::updateAlignment(const OdometryState& odometry) {
    const float phi_error = target_absolute_phi_rad_ - odometry.pose.absolute_phi_rad;
    const float distance_remaining_mm =
        std::fabs(phi_error) * config_.wheel_spacing_mm * 0.5f;

    status_.phi_error_rad = phi_error;
    status_.remaining_mm = distance_remaining_mm;

    if (std::fabs(phi_error) <= config_.orientation_tolerance_rad && robotStopped(odometry)) {
        if (go_to_active_) {
            // Alignment is complete. Reuse the normal straight controller for the second stage.
            const Point2D target = go_to_target_;
            const float speed = go_to_cruise_speed_mm_s_;
            const int direction = go_to_direction_;
            startStraightMovement(odometry.pose, target, speed, direction);
            go_to_active_ = true;
            return {};
        }

        movementFinished();
        return {};
    }

    const float distance_travelled_mm =
        std::max(0.0f, rotation_distance_mm_ - distance_remaining_mm);
    float wheel_speed = calculateProfileSpeed(distance_travelled_mm,
                                              distance_remaining_mm,
                                              rotating_speed_mm_s_,
                                              config_.rotating_ramping_acceleration_mm_s2,
                                              config_.rotating_breaking_acceleration_mm_s2,
                                              config_.rotating_minimum_speed_mm_s);

    if (distance_remaining_mm < 8.0f) {
        wheel_speed = std::min(wheel_speed, 120.0f);
    }

    const float angular_speed =
        sign(phi_error) * 2.0f * wheel_speed / config_.wheel_spacing_mm;
    return makeWheelSpeedCommand(0.0f, angular_speed);
}

WheelSpeeds Navigator::updateRotation(const OdometryState& odometry) {
    const float phi_error = target_absolute_phi_rad_ - odometry.pose.absolute_phi_rad;
    const float distance_remaining_mm =
        std::fabs(phi_error) * config_.wheel_spacing_mm * 0.5f;
    const float distance_travelled_mm =
        std::max(0.0f, rotation_distance_mm_ - distance_remaining_mm);

    status_.progress_mm = distance_travelled_mm;
    status_.remaining_mm = distance_remaining_mm;
    status_.phi_error_rad = phi_error;

    // The robot must stay inside tolerance for several control cycles. This avoids declaring
    // success during a single encoder/noise crossing of the target angle.
    if (std::fabs(phi_error) <= config_.phi_tolerance_rad && robotStopped(odometry)) {
        ++settle_counter_;
        if (settle_counter_ >= config_.settle_cycles) {
            movementFinished();
            return {};
        }
    } else {
        settle_counter_ = 0;
    }

    // Very near the final orientation, proportional angular control gives a smoother settle
    // than forcing the minimum rotation profile speed.
    if (distance_remaining_mm < 6.0f) {
        return makeWheelSpeedCommand(0.0f, config_.final_phi_kp * phi_error);
    }

    const float wheel_speed = calculateProfileSpeed(distance_travelled_mm,
                                                     distance_remaining_mm,
                                                     rotating_speed_mm_s_,
                                                     config_.rotating_ramping_acceleration_mm_s2,
                                                     config_.rotating_breaking_acceleration_mm_s2,
                                                     config_.rotating_minimum_speed_mm_s);
    const float angular_speed =
        sign(phi_error) * 2.0f * wheel_speed / config_.wheel_spacing_mm;

    return makeWheelSpeedCommand(0.0f, angular_speed);
}

WheelSpeeds Navigator::updateStraightMovement(const OdometryState& odometry) {
    const float from_start_x = odometry.pose.x_mm - starting_position_.x;
    const float from_start_y = odometry.pose.y_mm - starting_position_.y;

    // Projection on the commanded line gives travelled distance. The signed perpendicular
    // distance is the same idea as the lane correction term in the supplied STM32 code.
    const float distance_travelled_mm =
        from_start_x * direction_x_ + from_start_y * direction_y_;
    const float lane_error_mm =
        direction_x_ * from_start_y - direction_y_ * from_start_x;

    const float distance_remaining_on_line_mm = target_distance_mm_ - distance_travelled_mm;
    const float distance_to_target_mm = distance2D(target_position_.x - odometry.pose.x_mm,
                                                   target_position_.y - odometry.pose.y_mm);

    const float desired_phi =
        wrapAngle(starting_phi_rad_ + (direction_ < 0 ? kPi : 0.0f));
    const float phi_error = wrapAngle(desired_phi - odometry.pose.phi_rad);

    status_.progress_mm = clamp(distance_travelled_mm, 0.0f, target_distance_mm_);
    status_.remaining_mm = distance_to_target_mm;
    status_.lane_error_mm = lane_error_mm;
    status_.phi_error_rad = phi_error;

    const bool position_reached =
        distance_to_target_mm <= config_.position_tolerance_mm &&
        std::fabs(lane_error_mm) <= config_.lane_tolerance_mm;
    const bool phi_reached = std::fabs(phi_error) <= config_.phi_tolerance_rad;

    if (position_reached && phi_reached && robotStopped(odometry)) {
        ++settle_counter_;

        if (settle_counter_ >= config_.settle_cycles) {
            // go_to can optionally end with a requested final robot orientation.
            if (go_to_active_ && go_to_use_final_phi_) {
                go_to_active_ = false;
                target_absolute_phi_rad_ =
                    odometry.pose.absolute_phi_rad +
                    wrapAngle(go_to_final_phi_rad_ - odometry.pose.phi_rad);
                rotation_distance_mm_ =
                    std::fabs(target_absolute_phi_rad_ - odometry.pose.absolute_phi_rad) *
                    config_.wheel_spacing_mm * 0.5f;
                rotating_speed_mm_s_ = config_.rotating_speed_mm_s;
                settle_counter_ = 0;
                status_.mode = MotionMode::FINAL_ORIENT;
                return {};
            }

            movementFinished();
            return {};
        }
    } else {
        settle_counter_ = 0;
    }

    // Near the final point we steer directly to the endpoint instead of controlling only
    // distance along the original line. This prevents the robot from stopping beside the
    // target if wheel mismatch or floor slip creates a lateral offset.
    if (distance_to_target_mm < 45.0f) {
        if (position_reached) {
            return makeWheelSpeedCommand(0.0f, config_.final_phi_kp * phi_error);
        }

        const float endpoint_phi = std::atan2(target_position_.y - odometry.pose.y_mm,
                                              target_position_.x - odometry.pose.x_mm);
        const float desired_endpoint_phi =
            wrapAngle(endpoint_phi + (direction_ < 0 ? kPi : 0.0f));
        const float endpoint_phi_error =
            wrapAngle(desired_endpoint_phi - odometry.pose.phi_rad);

        const float phi_scale = clamp(std::cos(endpoint_phi_error), 0.2f, 1.0f);
        const float final_speed =
            std::min(config_.final_speed_limit_mm_s,
                     std::max(18.0f,
                              config_.final_position_kp_s_inv * distance_to_target_mm));

        return makeWheelSpeedCommand(static_cast<float>(direction_) * phi_scale * final_speed,
                                     config_.final_phi_kp * endpoint_phi_error);
    }

    const float speed = calculateProfileSpeed(
        std::max(0.0f, distance_travelled_mm),
        std::max(0.0f, distance_remaining_on_line_mm),
        cruise_speed_mm_s_,
        config_.ramping_acceleration_mm_s2,
        config_.breaking_acceleration_mm_s2,
        config_.minimum_cruise_speed_mm_s);

    const float signed_speed = static_cast<float>(direction_) * speed;

    // Lane correction combines phi error with a Stanley-style lateral correction. The
    // atan2 form remains well behaved near zero speed and does not require expensive path math.
    const float lane_correction_angle =
        std::atan2(config_.lane_correction_gain_s_inv * lane_error_mm,
                   std::fabs(signed_speed) + config_.lane_correction_softening_mm_s);

    const float angular_speed =
        config_.phi_correction_kp * phi_error -
        static_cast<float>(direction_) *
            config_.lane_correction_angle_gain * lane_correction_angle;

    return makeWheelSpeedCommand(signed_speed, angular_speed);
}

WheelSpeeds Navigator::update(const OdometryState& odometry, float /*dt_s*/) {
    switch (status_.mode) {
        case MotionMode::IDLE:
        case MotionMode::EMERGENCY_STOP:
        case MotionMode::FAULT:
            return {};

        case MotionMode::VELOCITY:
            return makeWheelSpeedCommand(commanded_speed_mm_s_, commanded_angular_speed_rad_s_);

        case MotionMode::ALIGN:
            return updateAlignment(odometry);

        case MotionMode::DRIVE_LINE:
            return updateStraightMovement(odometry);

        case MotionMode::ROTATE:
        case MotionMode::FINAL_ORIENT:
            return updateRotation(odometry);
    }

    emergencyBreak(FaultCode::INVALID_COMMAND);
    return {};
}

}  // namespace diffnav
