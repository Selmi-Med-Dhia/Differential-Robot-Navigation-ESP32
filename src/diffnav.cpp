#include "diffnav.hpp"

namespace diffnav {

WheelSpeeds bodyToWheels(float linear_mm_s, float angular_rad_s, float track_width_mm) {
    const float half_track_mm = 0.5f * track_width_mm;
    return {
        linear_mm_s + angular_rad_s * half_track_mm,
        linear_mm_s - angular_rad_s * half_track_mm,
    };
}

WheelSpeeds limitWheelSpeeds(WheelSpeeds target, float max_wheel_speed_mm_s) {
    const float largest = std::max(std::fabs(target.right_mm_s), std::fabs(target.left_mm_s));

    if (largest > max_wheel_speed_mm_s && largest > 1e-6f) {
        const float scale = max_wheel_speed_mm_s / largest;
        target.right_mm_s *= scale;
        target.left_mm_s *= scale;
    }

    return target;
}

float profileSpeed(float progress_mm,
                   float remaining_mm,
                   float cruise_speed_mm_s,
                   float acceleration_mm_s2,
                   float deceleration_mm_s2,
                   float minimum_speed_mm_s) {
    if (remaining_mm <= 0.0f || cruise_speed_mm_s <= 0.0f) {
        return 0.0f;
    }

    progress_mm = std::max(0.0f, progress_mm);
    remaining_mm = std::max(0.0f, remaining_mm);
    acceleration_mm_s2 = std::max(1.0f, acceleration_mm_s2);
    deceleration_mm_s2 = std::max(1.0f, deceleration_mm_s2);
    minimum_speed_mm_s = std::max(0.0f, minimum_speed_mm_s);

    const float acceleration_limited_speed =
        std::sqrt(minimum_speed_mm_s * minimum_speed_mm_s + 2.0f * acceleration_mm_s2 * progress_mm);
    const float braking_limited_speed = std::sqrt(2.0f * deceleration_mm_s2 * remaining_mm);

    float speed = std::min(cruise_speed_mm_s,
                           std::min(acceleration_limited_speed, braking_limited_speed));

    if (remaining_mm > 1.0f) {
        speed = std::max(speed, std::min(minimum_speed_mm_s, braking_limited_speed));
    }

    return std::max(0.0f, speed);
}

int32_t extendLegacyPcntDelta(int16_t previous, int16_t current, int32_t limit) {
    int32_t delta = static_cast<int32_t>(current) - static_cast<int32_t>(previous);
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

void WheelSpeedController::reset(float output_pwm) {
    integral_pwm_ = 0.0f;
    derivative_mm_s2_ = 0.0f;
    previous_measurement_mm_s_ = 0.0f;
    previous_output_pwm_ = output_pwm;
    initialized_ = false;
}

float WheelSpeedController::update(float target_mm_s, float measured_mm_s, float dt_s) {
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        return previous_output_pwm_;
    }

    if (std::fabs(target_mm_s) < 0.5f) {
        reset();
        return 0.0f;
    }

    if (!initialized_) {
        previous_measurement_mm_s_ = measured_mm_s;
        initialized_ = true;
    }

    const float error_mm_s = target_mm_s - measured_mm_s;
    const float raw_derivative = -(measured_mm_s - previous_measurement_mm_s_) / dt_s;
    const float derivative_alpha = dt_s / (config_.derivative_tau_s + dt_s);

    derivative_mm_s2_ += derivative_alpha * (raw_derivative - derivative_mm_s2_);
    previous_measurement_mm_s_ = measured_mm_s;

    const float feedforward_pwm =
        sign(target_mm_s) *
        (config_.static_feedforward_pwm +
         config_.velocity_feedforward_pwm_per_mm_s * std::fabs(target_mm_s));

    integral_pwm_ += config_.ki * error_mm_s * dt_s;
    integral_pwm_ = clamp(integral_pwm_, -config_.integral_limit_pwm, config_.integral_limit_pwm);

    const float unconstrained_pwm = feedforward_pwm +
                                    config_.kp * error_mm_s +
                                    integral_pwm_ +
                                    config_.kd * derivative_mm_s2_;

    float output_pwm = clamp(unconstrained_pwm, -config_.pwm_limit, config_.pwm_limit);

    integral_pwm_ += config_.antiwindup_gain_s_inv * (output_pwm - unconstrained_pwm) * dt_s;
    integral_pwm_ = clamp(integral_pwm_, -config_.integral_limit_pwm, config_.integral_limit_pwm);

    const float maximum_change = config_.pwm_slew_per_s * dt_s;
    output_pwm = clamp(output_pwm,
                       previous_output_pwm_ - maximum_change,
                       previous_output_pwm_ + maximum_change);

    previous_output_pwm_ = output_pwm;
    return output_pwm;
}

DifferentialOdometry::DifferentialOdometry(OdometryConfig config)
    : config_(config) {}

void DifferentialOdometry::reset(const Pose2D& pose, int64_t right_ticks, int64_t left_ticks) {
    state_ = {};
    state_.pose = pose;
    state_.pose.theta_rad = wrapAngle(pose.theta_rad);
    state_.pose.theta_unwrapped_rad = pose.theta_unwrapped_rad;
    state_.right_ticks = right_ticks;
    state_.left_ticks = left_ticks;

    previous_right_ticks_ = right_ticks;
    previous_left_ticks_ = left_ticks;
    initialized_ = true;
}

void DifferentialOdometry::setPose(const Pose2D& pose) {
    state_.pose = pose;
    state_.pose.theta_rad = wrapAngle(pose.theta_rad);
    state_.pose.theta_unwrapped_rad = pose.theta_unwrapped_rad;
}

float DifferentialOdometry::rightTicksToDistance(int64_t ticks) const {
    return static_cast<float>(ticks) * config_.right_wheel_diameter_mm * kPi /
           static_cast<float>(config_.counts_per_revolution);
}

float DifferentialOdometry::leftTicksToDistance(int64_t ticks) const {
    return static_cast<float>(ticks) * config_.left_wheel_diameter_mm * kPi /
           static_cast<float>(config_.counts_per_revolution);
}

int64_t DifferentialOdometry::rightDistanceToTicks(float distance_mm) const {
    return std::llround(distance_mm * config_.counts_per_revolution /
                        (config_.right_wheel_diameter_mm * kPi));
}

int64_t DifferentialOdometry::leftDistanceToTicks(float distance_mm) const {
    return std::llround(distance_mm * config_.counts_per_revolution /
                        (config_.left_wheel_diameter_mm * kPi));
}

void DifferentialOdometry::update(int64_t right_ticks, int64_t left_ticks, float dt_s) {
    if (!initialized_) {
        reset(state_.pose, right_ticks, left_ticks);
        return;
    }

    if (dt_s <= 0.0f || dt_s > 0.1f) {
        previous_right_ticks_ = right_ticks;
        previous_left_ticks_ = left_ticks;
        state_.right_ticks = right_ticks;
        state_.left_ticks = left_ticks;
        return;
    }

    const int64_t right_tick_delta = right_ticks - previous_right_ticks_;
    const int64_t left_tick_delta = left_ticks - previous_left_ticks_;

    previous_right_ticks_ = right_ticks;
    previous_left_ticks_ = left_ticks;

    const float right_distance_mm = rightTicksToDistance(right_tick_delta);
    const float left_distance_mm = leftTicksToDistance(left_tick_delta);
    const float center_distance_mm = 0.5f * (right_distance_mm + left_distance_mm);
    const float delta_theta_rad =
        (right_distance_mm - left_distance_mm) / config_.track_width_mm;

    float body_x_mm = 0.0f;
    float body_y_mm = 0.0f;

    if (std::fabs(delta_theta_rad) < 1e-6f) {
        body_x_mm = center_distance_mm;
    } else {
        const float turn_radius_mm = center_distance_mm / delta_theta_rad;
        body_x_mm = turn_radius_mm * std::sin(delta_theta_rad);
        body_y_mm = turn_radius_mm * (1.0f - std::cos(delta_theta_rad));
    }

    const float heading_rad = state_.pose.theta_unwrapped_rad;
    const float cos_heading = std::cos(heading_rad);
    const float sin_heading = std::sin(heading_rad);

    state_.pose.x_mm += cos_heading * body_x_mm - sin_heading * body_y_mm;
    state_.pose.y_mm += sin_heading * body_x_mm + cos_heading * body_y_mm;
    state_.pose.theta_unwrapped_rad += delta_theta_rad;
    state_.pose.theta_rad = wrapAngle(state_.pose.theta_unwrapped_rad);

    const float filter_alpha = dt_s / (config_.speed_filter_tau_s + dt_s);
    const float raw_right_speed = right_distance_mm / dt_s;
    const float raw_left_speed = left_distance_mm / dt_s;

    state_.wheel_speed.right_mm_s +=
        filter_alpha * (raw_right_speed - state_.wheel_speed.right_mm_s);
    state_.wheel_speed.left_mm_s +=
        filter_alpha * (raw_left_speed - state_.wheel_speed.left_mm_s);

    state_.body_velocity.linear_mm_s =
        0.5f * (state_.wheel_speed.right_mm_s + state_.wheel_speed.left_mm_s);
    state_.body_velocity.angular_rad_s =
        (state_.wheel_speed.right_mm_s - state_.wheel_speed.left_mm_s) /
        config_.track_width_mm;

    state_.right_ticks = right_ticks;
    state_.left_ticks = left_ticks;
}

Navigator::Navigator(NavigatorConfig config)
    : config_(config) {
    stop();
}

void Navigator::stop() {
    status_ = {};
    status_.mode = MotionMode::IDLE;
    status_.result = MotionResult::IDLE;

    velocity_linear_mm_s_ = 0.0f;
    velocity_angular_rad_s_ = 0.0f;
    goto_active_ = false;
    settle_counter_ = 0;
}

void Navigator::emergencyStop(FaultCode fault) {
    status_.mode = fault == FaultCode::NONE ? MotionMode::EMERGENCY_STOP : MotionMode::FAULT;
    status_.result = fault == FaultCode::NONE ? MotionResult::CANCELLED : MotionResult::FAULTED;
    status_.fault = fault;

    velocity_linear_mm_s_ = 0.0f;
    velocity_angular_rad_s_ = 0.0f;
    goto_active_ = false;
    settle_counter_ = 0;
}

void Navigator::clearEmergency() {
    stop();
}

void Navigator::commandVelocity(float linear_mm_s, float angular_rad_s) {
    if (status_.mode == MotionMode::EMERGENCY_STOP || status_.mode == MotionMode::FAULT) {
        return;
    }

    velocity_linear_mm_s_ = clamp(linear_mm_s,
                                  -config_.max_wheel_speed_mm_s,
                                  config_.max_wheel_speed_mm_s);
    velocity_angular_rad_s_ = clamp(angular_rad_s,
                                    -config_.max_angular_rate_rad_s,
                                    config_.max_angular_rate_rad_s);

    goto_active_ = false;
    settle_counter_ = 0;
    status_.mode = MotionMode::VELOCITY;
    status_.result = MotionResult::RUNNING;
    status_.fault = FaultCode::NONE;
}

void Navigator::beginLine(const Pose2D& current_pose,
                          Point2D target,
                          float speed_mm_s,
                          int direction) {
    line_start_ = {current_pose.x_mm, current_pose.y_mm};
    line_target_ = target;

    const float delta_x = line_target_.x - line_start_.x;
    const float delta_y = line_target_.y - line_start_.y;
    line_length_mm_ = distance2D(delta_x, delta_y);

    if (line_length_mm_ < 1e-3f) {
        markSucceeded();
        return;
    }

    line_unit_x_ = delta_x / line_length_mm_;
    line_unit_y_ = delta_y / line_length_mm_;
    line_heading_rad_ = std::atan2(delta_y, delta_x);
    line_direction_ = direction >= 0 ? 1 : -1;

    const float requested_speed = speed_mm_s > 0.0f
                                      ? speed_mm_s
                                      : config_.default_line_speed_mm_s;
    line_cruise_speed_mm_s_ = clamp(requested_speed,
                                    config_.line_min_speed_mm_s,
                                    config_.max_wheel_speed_mm_s);

    settle_counter_ = 0;
    status_.mode = MotionMode::DRIVE_LINE;
    status_.result = MotionResult::RUNNING;
    status_.fault = FaultCode::NONE;
}

void Navigator::commandMoveDistance(const Pose2D& current_pose,
                                    float distance_mm,
                                    float speed_mm_s) {
    Point2D target{
        current_pose.x_mm + distance_mm * std::cos(current_pose.theta_rad),
        current_pose.y_mm + distance_mm * std::sin(current_pose.theta_rad),
    };

    goto_active_ = false;
    beginLine(current_pose, target, speed_mm_s, distance_mm >= 0.0f ? 1 : -1);
}

void Navigator::commandRotateRelative(const Pose2D& current_pose,
                                      float angle_rad,
                                      float wheel_speed_mm_s) {
    goto_active_ = false;
    target_heading_unwrapped_rad_ = current_pose.theta_unwrapped_rad + angle_rad;
    rotation_length_mm_ = std::fabs(angle_rad) * config_.track_width_mm * 0.5f;
    rotation_wheel_speed_mm_s_ = wheel_speed_mm_s > 0.0f
                                     ? std::min(wheel_speed_mm_s, config_.max_wheel_speed_mm_s)
                                     : config_.rotate_wheel_speed_mm_s;

    settle_counter_ = 0;
    status_.mode = MotionMode::ROTATE;
    status_.result = MotionResult::RUNNING;
    status_.fault = FaultCode::NONE;
}

void Navigator::commandOrientAbsolute(const Pose2D& current_pose,
                                      float angle_rad,
                                      float wheel_speed_mm_s) {
    const float shortest_angle = wrapAngle(angle_rad - current_pose.theta_rad);
    commandRotateRelative(current_pose, shortest_angle, wheel_speed_mm_s);
}

void Navigator::commandGoTo(const Pose2D& current_pose,
                            float target_x_mm,
                            float target_y_mm,
                            float speed_mm_s,
                            int direction,
                            bool use_final_heading,
                            float final_heading_rad) {
    const float delta_x = target_x_mm - current_pose.x_mm;
    const float delta_y = target_y_mm - current_pose.y_mm;

    if (distance2D(delta_x, delta_y) <= config_.position_tolerance_mm) {
        if (use_final_heading) {
            commandOrientAbsolute(current_pose, final_heading_rad);
        } else {
            markSucceeded();
        }
        return;
    }

    goto_active_ = true;
    goto_target_ = {target_x_mm, target_y_mm};
    goto_speed_mm_s_ = speed_mm_s > 0.0f ? speed_mm_s : config_.default_line_speed_mm_s;
    goto_direction_ = direction >= 0 ? 1 : -1;
    goto_use_final_heading_ = use_final_heading;
    goto_final_heading_rad_ = wrapAngle(final_heading_rad);

    const float path_heading = std::atan2(delta_y, delta_x);
    const float desired_heading = wrapAngle(path_heading + (goto_direction_ < 0 ? kPi : 0.0f));
    const float heading_delta = wrapAngle(desired_heading - current_pose.theta_rad);

    target_heading_unwrapped_rad_ = current_pose.theta_unwrapped_rad + heading_delta;
    rotation_length_mm_ = std::fabs(heading_delta) * config_.track_width_mm * 0.5f;
    rotation_wheel_speed_mm_s_ = config_.rotate_wheel_speed_mm_s;

    settle_counter_ = 0;
    status_.mode = MotionMode::ALIGN;
    status_.result = MotionResult::RUNNING;
    status_.fault = FaultCode::NONE;
}

WheelSpeeds Navigator::bodyCommand(float linear_mm_s, float angular_rad_s) const {
    const float limited_angular_rate = clamp(angular_rad_s,
                                             -config_.max_angular_rate_rad_s,
                                             config_.max_angular_rate_rad_s);

    return limitWheelSpeeds(
        bodyToWheels(linear_mm_s, limited_angular_rate, config_.track_width_mm),
        config_.max_wheel_speed_mm_s);
}

bool Navigator::robotStopped(const OdometryState& odometry) const {
    return std::fabs(odometry.wheel_speed.right_mm_s) < config_.stopped_speed_tolerance_mm_s &&
           std::fabs(odometry.wheel_speed.left_mm_s) < config_.stopped_speed_tolerance_mm_s;
}

void Navigator::markSucceeded() {
    status_.mode = MotionMode::IDLE;
    status_.result = MotionResult::SUCCEEDED;
    status_.fault = FaultCode::NONE;
    goto_active_ = false;
    settle_counter_ = 0;
}

WheelSpeeds Navigator::updateAlignment(const OdometryState& odometry) {
    const float heading_error =
        target_heading_unwrapped_rad_ - odometry.pose.theta_unwrapped_rad;
    const float remaining_mm =
        std::fabs(heading_error) * config_.track_width_mm * 0.5f;

    status_.heading_error_rad = heading_error;
    status_.remaining_mm = remaining_mm;

    if (std::fabs(heading_error) <= config_.align_tolerance_rad && robotStopped(odometry)) {
        if (goto_active_) {
            const Point2D target = goto_target_;
            const float speed = goto_speed_mm_s_;
            const int direction = goto_direction_;

            beginLine(odometry.pose, target, speed, direction);
            goto_active_ = true;
            return {};
        }

        markSucceeded();
        return {};
    }

    const float progress_mm = std::max(0.0f, rotation_length_mm_ - remaining_mm);
    float wheel_speed_mm_s = profileSpeed(progress_mm,
                                          remaining_mm,
                                          rotation_wheel_speed_mm_s_,
                                          config_.rotate_accel_mm_s2,
                                          config_.rotate_decel_mm_s2,
                                          config_.rotate_min_wheel_speed_mm_s);

    if (remaining_mm < 8.0f) {
        wheel_speed_mm_s = std::min(wheel_speed_mm_s, 120.0f);
    }

    const float angular_rate =
        sign(heading_error) * 2.0f * wheel_speed_mm_s / config_.track_width_mm;
    return bodyCommand(0.0f, angular_rate);
}

WheelSpeeds Navigator::updateRotation(const OdometryState& odometry) {
    const float heading_error =
        target_heading_unwrapped_rad_ - odometry.pose.theta_unwrapped_rad;
    const float remaining_mm =
        std::fabs(heading_error) * config_.track_width_mm * 0.5f;
    const float progress_mm = std::max(0.0f, rotation_length_mm_ - remaining_mm);

    status_.progress_mm = progress_mm;
    status_.remaining_mm = remaining_mm;
    status_.heading_error_rad = heading_error;

    if (std::fabs(heading_error) <= config_.heading_tolerance_rad && robotStopped(odometry)) {
        ++settle_counter_;
        if (settle_counter_ >= config_.settle_cycles) {
            markSucceeded();
            return {};
        }
    } else {
        settle_counter_ = 0;
    }

    if (remaining_mm < 6.0f) {
        return bodyCommand(0.0f, config_.final_heading_kp * heading_error);
    }

    const float wheel_speed_mm_s = profileSpeed(progress_mm,
                                                remaining_mm,
                                                rotation_wheel_speed_mm_s_,
                                                config_.rotate_accel_mm_s2,
                                                config_.rotate_decel_mm_s2,
                                                config_.rotate_min_wheel_speed_mm_s);

    const float angular_rate =
        sign(heading_error) * 2.0f * wheel_speed_mm_s / config_.track_width_mm;
    return bodyCommand(0.0f, angular_rate);
}

WheelSpeeds Navigator::updateLine(const OdometryState& odometry) {
    const float from_start_x = odometry.pose.x_mm - line_start_.x;
    const float from_start_y = odometry.pose.y_mm - line_start_.y;

    const float along_track_mm =
        from_start_x * line_unit_x_ + from_start_y * line_unit_y_;
    const float cross_track_mm =
        line_unit_x_ * from_start_y - line_unit_y_ * from_start_x;

    const float along_remaining_mm = line_length_mm_ - along_track_mm;
    const float endpoint_distance_mm = distance2D(line_target_.x - odometry.pose.x_mm,
                                                  line_target_.y - odometry.pose.y_mm);

    const float desired_heading =
        wrapAngle(line_heading_rad_ + (line_direction_ < 0 ? kPi : 0.0f));
    const float heading_error = wrapAngle(desired_heading - odometry.pose.theta_rad);

    status_.progress_mm = clamp(along_track_mm, 0.0f, line_length_mm_);
    status_.remaining_mm = endpoint_distance_mm;
    status_.cross_track_mm = cross_track_mm;
    status_.heading_error_rad = heading_error;

    const bool position_good =
        endpoint_distance_mm <= config_.position_tolerance_mm &&
        std::fabs(cross_track_mm) <= config_.cross_track_tolerance_mm;
    const bool heading_good = std::fabs(heading_error) <= config_.heading_tolerance_rad;

    if (position_good && heading_good && robotStopped(odometry)) {
        ++settle_counter_;

        if (settle_counter_ >= config_.settle_cycles) {
            if (goto_active_ && goto_use_final_heading_) {
                goto_active_ = false;
                target_heading_unwrapped_rad_ =
                    odometry.pose.theta_unwrapped_rad +
                    wrapAngle(goto_final_heading_rad_ - odometry.pose.theta_rad);
                rotation_length_mm_ =
                    std::fabs(target_heading_unwrapped_rad_ - odometry.pose.theta_unwrapped_rad) *
                    config_.track_width_mm * 0.5f;
                rotation_wheel_speed_mm_s_ = config_.rotate_wheel_speed_mm_s;
                settle_counter_ = 0;
                status_.mode = MotionMode::FINAL_ORIENT;
                return {};
            }

            markSucceeded();
            return {};
        }
    } else {
        settle_counter_ = 0;
    }

    // Close to the endpoint, stop caring about along-track error and home directly
    // on the final point. This avoids getting stuck beside the target.
    if (endpoint_distance_mm < 45.0f) {
        if (position_good) {
            return bodyCommand(0.0f, config_.final_heading_kp * heading_error);
        }

        const float endpoint_heading =
            std::atan2(line_target_.y - odometry.pose.y_mm,
                       line_target_.x - odometry.pose.x_mm);
        const float desired_endpoint_heading =
            wrapAngle(endpoint_heading + (line_direction_ < 0 ? kPi : 0.0f));
        const float endpoint_heading_error =
            wrapAngle(desired_endpoint_heading - odometry.pose.theta_rad);

        const float heading_scale = clamp(std::cos(endpoint_heading_error), 0.2f, 1.0f);
        const float commanded_speed =
            std::min(config_.final_speed_limit_mm_s,
                     std::max(18.0f,
                              config_.final_position_kp_s_inv * endpoint_distance_mm));

        return bodyCommand(static_cast<float>(line_direction_) * heading_scale * commanded_speed,
                           config_.final_heading_kp * endpoint_heading_error);
    }

    const float speed_mm_s = profileSpeed(std::max(0.0f, along_track_mm),
                                          std::max(0.0f, along_remaining_mm),
                                          line_cruise_speed_mm_s_,
                                          config_.line_accel_mm_s2,
                                          config_.line_decel_mm_s2,
                                          config_.line_min_speed_mm_s);

    const float linear_mm_s = static_cast<float>(line_direction_) * speed_mm_s;
    const float cross_track_angle =
        std::atan2(config_.cross_track_gain_s_inv * cross_track_mm,
                   std::fabs(linear_mm_s) + config_.stanley_softening_mm_s);

    const float angular_rad_s =
        config_.heading_kp * heading_error -
        static_cast<float>(line_direction_) * config_.stanley_gain * cross_track_angle;

    return bodyCommand(linear_mm_s, angular_rad_s);
}

WheelSpeeds Navigator::update(const OdometryState& odometry, float /*dt_s*/) {
    switch (status_.mode) {
        case MotionMode::IDLE:
        case MotionMode::EMERGENCY_STOP:
        case MotionMode::FAULT:
            return {};

        case MotionMode::VELOCITY:
            return bodyCommand(velocity_linear_mm_s_, velocity_angular_rad_s_);

        case MotionMode::ALIGN:
            return updateAlignment(odometry);

        case MotionMode::DRIVE_LINE:
            return updateLine(odometry);

        case MotionMode::ROTATE:
        case MotionMode::FINAL_ORIENT:
            return updateRotation(odometry);
    }

    emergencyStop(FaultCode::INVALID_COMMAND);
    return {};
}

}  // namespace diffnav
