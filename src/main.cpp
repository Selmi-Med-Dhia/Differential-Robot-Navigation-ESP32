#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "communication.hpp"
#include "diffnav.hpp"
#include "encoder_pcnt.hpp"
#include "motor_driver.hpp"
#include "robot_config.hpp"

namespace {

EncoderPcnt encoders;
MotorDriver motors;

diffnav::DifferentialOdometry odometry(robot_config::odometryConfig());
diffnav::Navigator navigator(robot_config::navigatorConfig());
diffnav::WheelSpeedController right_wheel_controller(robot_config::wheelControllerConfig());
diffnav::WheelSpeedController left_wheel_controller(robot_config::wheelControllerConfig());

QueueHandle_t command_queue = nullptr;
QueueHandle_t telemetry_queue = nullptr;
CommunicationContext communication_context{};

uint32_t right_stall_time_ms = 0;
uint32_t left_stall_time_ms = 0;

void processCommand(const diffnav::NavCommand& command) {
    const diffnav::Pose2D current_pose = odometry.state().pose;

    switch (command.type) {
        case diffnav::NavCommandType::STOP:
            navigator.stop();
            break;

        case diffnav::NavCommandType::EMERGENCY_STOP:
            navigator.emergencyStop();
            break;

        case diffnav::NavCommandType::CLEAR_EMERGENCY:
            navigator.clearEmergency();
            right_wheel_controller.reset();
            left_wheel_controller.reset();
            break;

        case diffnav::NavCommandType::VELOCITY:
            navigator.commandVelocity(command.p0, command.p1);
            break;

        case diffnav::NavCommandType::GO_TO:
            navigator.commandGoTo(current_pose,
                                  command.p0,
                                  command.p1,
                                  command.p2,
                                  command.direction,
                                  command.use_final_heading,
                                  command.p3);
            break;

        case diffnav::NavCommandType::MOVE_DISTANCE:
            navigator.commandMoveDistance(current_pose, command.p0, command.p1);
            break;

        case diffnav::NavCommandType::ROTATE_RELATIVE:
            navigator.commandRotateRelative(current_pose, command.p0, command.p1);
            break;

        case diffnav::NavCommandType::ORIENT_ABSOLUTE:
            navigator.commandOrientAbsolute(current_pose, command.p0, command.p1);
            break;

        case diffnav::NavCommandType::SET_POSE: {
            diffnav::Pose2D new_pose;
            new_pose.x_mm = command.p0;
            new_pose.y_mm = command.p1;
            new_pose.theta_rad = command.p2;
            new_pose.theta_unwrapped_rad = command.p2;
            odometry.setPose(new_pose);
            navigator.stop();
            break;
        }

        case diffnav::NavCommandType::NONE:
            break;
    }
}

bool updateStallDetector(float target_speed_mm_s,
                         float measured_speed_mm_s,
                         float pwm,
                         uint32_t elapsed_ms,
                         uint32_t& accumulated_ms) {
    const bool should_be_moving =
        std::fabs(target_speed_mm_s) >= robot_config::kStallTargetThresholdMmS;
    const bool is_not_moving =
        std::fabs(measured_speed_mm_s) <= robot_config::kStallMeasuredThresholdMmS;
    const bool motor_is_driven =
        std::fabs(pwm) >= robot_config::kStallPwmThreshold;

    if (should_be_moving && is_not_moving && motor_is_driven) {
        accumulated_ms += elapsed_ms;
    } else {
        accumulated_ms = 0;
    }

    return accumulated_ms >= robot_config::kStallTimeoutMs;
}

void publishTelemetry(const diffnav::WheelSpeeds& target,
                      float right_pwm,
                      float left_pwm) {
    TelemetryFrame frame;
    frame.odometry = odometry.state();
    frame.motion = navigator.status();
    frame.target_wheel_speed = target;
    frame.right_pwm = right_pwm;
    frame.left_pwm = left_pwm;

    xQueueOverwrite(telemetry_queue, &frame);
}

void navigationControlTask(void*) {
    EncoderCounts counts = encoders.snapshot();
    odometry.reset({}, counts.right, counts.left);

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t previous_time_us = micros();
    uint32_t telemetry_divider = 0;
    uint8_t bad_timing_cycles = 0;

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(robot_config::kControlPeriodUs / 1000));

        const uint32_t now_us = micros();
        const uint32_t elapsed_us = now_us - previous_time_us;
        previous_time_us = now_us;
        const float dt_s = elapsed_us * 1e-6f;

        if (elapsed_us > robot_config::kControlTimingFaultThresholdUs) {
            ++bad_timing_cycles;
            if (bad_timing_cycles >= robot_config::kControlTimingFaultCycles) {
                navigator.emergencyStop(diffnav::FaultCode::CONTROL_TIMING);
            }
        } else {
            bad_timing_cycles = 0;
        }

        counts = encoders.snapshot();
        odometry.update(counts.right, counts.left, dt_s);

        diffnav::NavCommand command;
        while (xQueueReceive(command_queue, &command, 0) == pdTRUE) {
            processCommand(command);
        }

        const diffnav::WheelSpeeds target = navigator.update(odometry.state(), dt_s);

        float right_pwm = right_wheel_controller.update(
            target.right_mm_s,
            odometry.state().wheel_speed.right_mm_s,
            dt_s);
        float left_pwm = left_wheel_controller.update(
            target.left_mm_s,
            odometry.state().wheel_speed.left_mm_s,
            dt_s);

        const uint32_t elapsed_ms = std::max<uint32_t>(1, elapsed_us / 1000);
        const bool right_stalled = updateStallDetector(
            target.right_mm_s,
            odometry.state().wheel_speed.right_mm_s,
            right_pwm,
            elapsed_ms,
            right_stall_time_ms);
        const bool left_stalled = updateStallDetector(
            target.left_mm_s,
            odometry.state().wheel_speed.left_mm_s,
            left_pwm,
            elapsed_ms,
            left_stall_time_ms);

        if (right_stalled || left_stalled) {
            navigator.emergencyStop(right_stalled
                                        ? diffnav::FaultCode::STALL_RIGHT
                                        : diffnav::FaultCode::STALL_LEFT);
            right_wheel_controller.reset();
            left_wheel_controller.reset();
            right_pwm = 0.0f;
            left_pwm = 0.0f;
        }

        const diffnav::MotionMode mode = navigator.mode();
        if (mode == diffnav::MotionMode::IDLE ||
            mode == diffnav::MotionMode::EMERGENCY_STOP ||
            mode == diffnav::MotionMode::FAULT) {
            right_pwm = 0.0f;
            left_pwm = 0.0f;
            right_wheel_controller.reset();
            left_wheel_controller.reset();
        }

        motors.write(right_pwm, left_pwm);

        ++telemetry_divider;
        if (telemetry_divider >= 8) {
            telemetry_divider = 0;
            publishTelemetry(target, right_pwm, left_pwm);
        }
    }
}

}  // namespace

void setup() {
    // micro-ROS uses Serial. UART mode uses Serial2 for commands and can leave Serial
    // available for a USB terminal if desired.
    Serial.begin(115200);

    command_queue = xQueueCreate(8, sizeof(diffnav::NavCommand));
    telemetry_queue = xQueueCreate(1, sizeof(TelemetryFrame));

    if (command_queue == nullptr ||
        telemetry_queue == nullptr ||
        !motors.begin() ||
        !encoders.begin()) {
        motors.stop();
        while (true) {
            delay(1000);
        }
    }

    communication_context.command_queue = command_queue;
    communication_context.telemetry_queue = telemetry_queue;

    xTaskCreatePinnedToCore(navigationControlTask,
                            "navigation_control",
                            6144,
                            nullptr,
                            4,
                            nullptr,
                            1);

    xTaskCreatePinnedToCore(communicationTask,
                            "communication",
                            8192,
                            &communication_context,
                            1,
                            nullptr,
                            0);
}

void loop() {
    delay(1000);
}
