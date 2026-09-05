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
diffnav::WheelSpeedController speed_controller_R(robot_config::wheelControllerConfig());
diffnav::WheelSpeedController speed_controller_L(robot_config::wheelControllerConfig());

QueueHandle_t command_queue = nullptr;
QueueHandle_t telemetry_queue = nullptr;
CommunicationContext communication_context{};

uint32_t stall_time_R_ms = 0;
uint32_t stall_time_L_ms = 0;

void processCommand(const diffnav::NavCommand& command) {
    const diffnav::Pose2D current_pose = odometry.state().pose;

    // Communication backends only create NavCommand objects. All actual navigation commands
    // are executed here, inside the control side of the program.
    switch (command.type) {
        case diffnav::NavCommandType::STOP:
            navigator.stop();
            break;

        case diffnav::NavCommandType::EMERGENCY_STOP:
            navigator.emergencyBreak();
            break;

        case diffnav::NavCommandType::CLEAR_EMERGENCY:
            navigator.clearEmergencyBreak();
            speed_controller_R.reset();
            speed_controller_L.reset();
            break;

        case diffnav::NavCommandType::VELOCITY:
            navigator.setSpeed(command.speed_mm_s, command.angular_speed_rad_s);
            break;

        case diffnav::NavCommandType::GO_TO:
            navigator.goTo(current_pose,
                           command.x_mm,
                           command.y_mm,
                           command.speed_mm_s,
                           command.direction,
                           command.use_final_phi,
                           command.phi_rad);
            break;

        case diffnav::NavCommandType::MOVE_DISTANCE:
            navigator.moveForward(current_pose, command.distance_mm, command.speed_mm_s);
            break;

        case diffnav::NavCommandType::ROTATE_RELATIVE:
            navigator.rotate(current_pose, command.phi_rad, command.speed_mm_s);
            break;

        case diffnav::NavCommandType::ORIENT_ABSOLUTE:
            navigator.orientate(current_pose, command.phi_rad, command.speed_mm_s);
            break;

        case diffnav::NavCommandType::SET_POSE: {
            diffnav::Pose2D new_pose;
            new_pose.x_mm = command.x_mm;
            new_pose.y_mm = command.y_mm;
            new_pose.phi_rad = command.phi_rad;
            new_pose.absolute_phi_rad = command.phi_rad;
            odometry.setPose(new_pose);
            navigator.stop();
            break;
        }

        case diffnav::NavCommandType::NONE:
            break;
    }
}

bool wheelIsStalled(float target_speed_mm_s,
                    float measured_speed_mm_s,
                    float pwm,
                    uint32_t elapsed_ms,
                    uint32_t& accumulated_stall_time_ms) {
    const bool should_be_moving =
        std::fabs(target_speed_mm_s) >= robot_config::stall_target_speed_threshold_mm_s;
    const bool encoder_says_stopped =
        std::fabs(measured_speed_mm_s) <= robot_config::stall_measured_speed_threshold_mm_s;
    const bool motor_has_enough_pwm = std::fabs(pwm) >= robot_config::stall_pwm_threshold;

    if (should_be_moving && encoder_says_stopped && motor_has_enough_pwm) {
        accumulated_stall_time_ms += elapsed_ms;
    } else {
        accumulated_stall_time_ms = 0;
    }

    return accumulated_stall_time_ms >= robot_config::stall_timeout_ms;
}

void sendTelemetryToCommunicationTask(const diffnav::WheelSpeeds& target_speed,
                                      float pwm_R,
                                      float pwm_L) {
    TelemetryFrame frame;
    frame.odometry = odometry.state();
    frame.motion = navigator.status();
    frame.target_wheel_speed = target_speed;
    frame.pwm_R = pwm_R;
    frame.pwm_L = pwm_L;

    // The queue has one element. Overwrite means communication always gets the newest state
    // and can never block the real-time control task waiting for old telemetry to be consumed.
    xQueueOverwrite(telemetry_queue, &frame);
}

void navigationControlTask(void*) {
    EncoderCounts encoder_counts = encoders.snapshot();
    odometry.reset({}, encoder_counts.encoder_count_R, encoder_counts.encoder_count_L);

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t previous_time_us = micros();
    uint32_t telemetry_divider = 0;
    uint8_t bad_timing_cycles = 0;

    for (;;) {
        // vTaskDelayUntil produces a fixed-rate control loop instead of accumulating delay
        // error from one iteration to the next.
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(robot_config::control_period_us / 1000));

        const uint32_t now_us = micros();
        const uint32_t elapsed_us = now_us - previous_time_us;
        previous_time_us = now_us;
        const float dt_s = elapsed_us * 1e-6f;

        // Repeated large scheduling delays are treated as a control fault. One isolated late
        // cycle is tolerated because Wi-Fi/USB/RTOS activity can occasionally add jitter.
        if (elapsed_us > robot_config::control_timing_fault_threshold_us) {
            ++bad_timing_cycles;
            if (bad_timing_cycles >= robot_config::control_timing_fault_cycles) {
                navigator.emergencyBreak(diffnav::FaultCode::CONTROL_TIMING);
            }
        } else {
            bad_timing_cycles = 0;
        }

        // 1) Read hardware encoder counts and update pose + measured wheel speeds.
        encoder_counts = encoders.snapshot();
        odometry.update(encoder_counts.encoder_count_R,
                        encoder_counts.encoder_count_L,
                        dt_s);

        // 2) Apply every command that arrived since the previous control cycle.
        diffnav::NavCommand command;
        while (xQueueReceive(command_queue, &command, 0) == pdTRUE) {
            processCommand(command);
        }

        // 3) Navigation converts the current robot state into desired right/left wheel speeds.
        const diffnav::WheelSpeeds target_speed = navigator.update(odometry.state(), dt_s);

        // 4) Independent wheel speed controllers convert desired speed into motor PWM.
        float pwm_R = speed_controller_R.update(
            target_speed.speed_R_mm_s,
            odometry.state().wheel_speed.speed_R_mm_s,
            dt_s);
        float pwm_L = speed_controller_L.update(
            target_speed.speed_L_mm_s,
            odometry.state().wheel_speed.speed_L_mm_s,
            dt_s);

        // 5) Detect a blocked wheel: high requested speed + high PWM + almost no encoder motion.
        const uint32_t elapsed_ms = std::max<uint32_t>(1, elapsed_us / 1000);
        const bool stalled_R = wheelIsStalled(target_speed.speed_R_mm_s,
                                              odometry.state().wheel_speed.speed_R_mm_s,
                                              pwm_R,
                                              elapsed_ms,
                                              stall_time_R_ms);
        const bool stalled_L = wheelIsStalled(target_speed.speed_L_mm_s,
                                              odometry.state().wheel_speed.speed_L_mm_s,
                                              pwm_L,
                                              elapsed_ms,
                                              stall_time_L_ms);

        if (stalled_R || stalled_L) {
            navigator.emergencyBreak(stalled_R
                                         ? diffnav::FaultCode::STALL_RIGHT
                                         : diffnav::FaultCode::STALL_LEFT);
            speed_controller_R.reset();
            speed_controller_L.reset();
            pwm_R = 0.0f;
            pwm_L = 0.0f;
        }

        // Never leave residual controller output active when navigation is not moving.
        const diffnav::MotionMode mode = navigator.mode();
        if (mode == diffnav::MotionMode::IDLE ||
            mode == diffnav::MotionMode::EMERGENCY_STOP ||
            mode == diffnav::MotionMode::FAULT) {
            pwm_R = 0.0f;
            pwm_L = 0.0f;
            speed_controller_R.reset();
            speed_controller_L.reset();
        }

        motors.write(pwm_R, pwm_L);

        // Control runs at 200 Hz; telemetry only needs 25 Hz internally.
        if (++telemetry_divider >= 8) {
            telemetry_divider = 0;
            sendTelemetryToCommunicationTask(target_speed, pwm_R, pwm_L);
        }
    }
}

TaskFunction_t selectedCommunicationTask() {
    // This is the only selection point. Both backends are compiled into the same firmware;
    // robot_config::communication_type decides which task is started.
    if (robot_config::communication_type == robot_config::CommunicationType::UART) {
        return uartCommunicationTask;
    }
    return microRosCommunicationTask;
}

const char* selectedCommunicationTaskName() {
    return robot_config::communication_type == robot_config::CommunicationType::UART
               ? "uart_communication"
               : "micro_ros";
}

uint32_t selectedCommunicationStackSize() {
    return robot_config::communication_type == robot_config::CommunicationType::UART
               ? 4096
               : 8192;
}

}  // namespace

void setup() {
    // micro-ROS serial transport uses Serial. In UART mode Serial remains available for USB
    // debugging while robot commands use Serial2.
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

    // Core 1 is reserved for deterministic navigation/control.
    xTaskCreatePinnedToCore(navigationControlTask,
                            "navigation_control",
                            6144,
                            nullptr,
                            4,
                            nullptr,
                            1);

    // Core 0 runs the communication method selected in robot_config.hpp.
    xTaskCreatePinnedToCore(selectedCommunicationTask(),
                            selectedCommunicationTaskName(),
                            selectedCommunicationStackSize(),
                            &communication_context,
                            1,
                            nullptr,
                            0);
}

void loop() {
    // All work is done in FreeRTOS tasks.
    delay(1000);
}
