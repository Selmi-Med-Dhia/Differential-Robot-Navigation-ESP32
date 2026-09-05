#include "communication.hpp"

#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>

#include "robot_config.hpp"

namespace {

QueueHandle_t command_queue = nullptr;
QueueHandle_t telemetry_queue = nullptr;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t pose_publisher;
rcl_subscription_t velocity_subscription;
rcl_subscription_t goal_subscription;
rcl_subscription_t emergency_stop_subscription;
rclc_executor_t executor;

geometry_msgs__msg__Pose2D pose_message;
geometry_msgs__msg__Pose2D goal_message;
geometry_msgs__msg__Twist velocity_message;
std_msgs__msg__Bool emergency_stop_message;

bool communication_ready = false;
bool support_initialized = false;
bool node_initialized = false;
bool publisher_initialized = false;
bool velocity_subscription_initialized = false;
bool goal_subscription_initialized = false;
bool emergency_subscription_initialized = false;
bool executor_initialized = false;

void enqueueCommand(const diffnav::NavCommand& command) {
    if (command_queue == nullptr) {
        return;
    }

    if (xQueueSend(command_queue, &command, 0) == pdTRUE) {
        return;
    }

    // If commands arrive faster than the control task can consume them, discard the oldest
    // command and keep the newest one.
    diffnav::NavCommand discarded;
    xQueueReceive(command_queue, &discarded, 0);
    xQueueSend(command_queue, &command, 0);
}

void velocityCallback(const void* message) {
    const auto* twist = static_cast<const geometry_msgs__msg__Twist*>(message);

    diffnav::NavCommand command;
    command.type = diffnav::NavCommandType::VELOCITY;
    command.speed_mm_s = static_cast<float>(twist->linear.x * 1000.0);  // m/s -> mm/s
    command.angular_speed_rad_s = static_cast<float>(twist->angular.z);
    enqueueCommand(command);
}

void goalCallback(const void* message) {
    const auto* goal = static_cast<const geometry_msgs__msg__Pose2D*>(message);

    diffnav::NavCommand command;
    command.type = diffnav::NavCommandType::GO_TO;
    command.x_mm = static_cast<float>(goal->x * 1000.0);  // m -> mm
    command.y_mm = static_cast<float>(goal->y * 1000.0);  // m -> mm
    command.speed_mm_s = 0.0f;                            // use default cruise speed
    command.phi_rad = static_cast<float>(goal->theta);
    command.direction = 1;
    command.use_final_phi = true;
    enqueueCommand(command);
}

void emergencyStopCallback(const void* message) {
    const auto* emergency = static_cast<const std_msgs__msg__Bool*>(message);

    diffnav::NavCommand command;
    command.type = emergency->data
                       ? diffnav::NavCommandType::EMERGENCY_STOP
                       : diffnav::NavCommandType::CLEAR_EMERGENCY;
    enqueueCommand(command);
}

void destroyEntities() {
    if (support_initialized) {
        rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
        if (rmw_context != nullptr) {
            rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    if (executor_initialized) {
        (void)rclc_executor_fini(&executor);
    }
    if (emergency_subscription_initialized && node_initialized) {
        (void)rcl_subscription_fini(&emergency_stop_subscription, &node);
    }
    if (goal_subscription_initialized && node_initialized) {
        (void)rcl_subscription_fini(&goal_subscription, &node);
    }
    if (velocity_subscription_initialized && node_initialized) {
        (void)rcl_subscription_fini(&velocity_subscription, &node);
    }
    if (publisher_initialized && node_initialized) {
        (void)rcl_publisher_fini(&pose_publisher, &node);
    }
    if (node_initialized) {
        (void)rcl_node_fini(&node);
    }
    if (support_initialized) {
        (void)rclc_support_fini(&support);
    }

    communication_ready = false;
    support_initialized = false;
    node_initialized = false;
    publisher_initialized = false;
    velocity_subscription_initialized = false;
    goal_subscription_initialized = false;
    emergency_subscription_initialized = false;
    executor_initialized = false;
}

bool createEntities() {
    allocator = rcl_get_default_allocator();
    support = {};
    node = rcl_get_zero_initialized_node();
    pose_publisher = rcl_get_zero_initialized_publisher();
    velocity_subscription = rcl_get_zero_initialized_subscription();
    goal_subscription = rcl_get_zero_initialized_subscription();
    emergency_stop_subscription = rcl_get_zero_initialized_subscription();
    executor = rclc_executor_get_zero_initialized_executor();

    if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
        return false;
    }
    support_initialized = true;

    if (rclc_node_init_default(&node, "diffnav_esp32", "", &support) != RCL_RET_OK) {
        return false;
    }
    node_initialized = true;

    if (rclc_publisher_init_best_effort(
            &pose_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
            "/robot_pose_raw") != RCL_RET_OK) {
        return false;
    }
    publisher_initialized = true;

    if (rclc_subscription_init_best_effort(
            &velocity_subscription,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK) {
        return false;
    }
    velocity_subscription_initialized = true;

    if (rclc_subscription_init_default(
            &goal_subscription,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
            "/nav/goal") != RCL_RET_OK) {
        return false;
    }
    goal_subscription_initialized = true;

    if (rclc_subscription_init_default(
            &emergency_stop_subscription,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/nav/emergency_stop") != RCL_RET_OK) {
        return false;
    }
    emergency_subscription_initialized = true;

    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) {
        return false;
    }
    executor_initialized = true;

    if (rclc_executor_add_subscription(&executor,
                                       &velocity_subscription,
                                       &velocity_message,
                                       velocityCallback,
                                       ON_NEW_DATA) != RCL_RET_OK ||
        rclc_executor_add_subscription(&executor,
                                       &goal_subscription,
                                       &goal_message,
                                       goalCallback,
                                       ON_NEW_DATA) != RCL_RET_OK ||
        rclc_executor_add_subscription(&executor,
                                       &emergency_stop_subscription,
                                       &emergency_stop_message,
                                       emergencyStopCallback,
                                       ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }

    communication_ready = true;
    return true;
}

void publishPose(const TelemetryFrame& frame) {
    pose_message.x = frame.odometry.pose.x_mm / 1000.0;
    pose_message.y = frame.odometry.pose.y_mm / 1000.0;
    pose_message.theta = frame.odometry.pose.phi_rad;
    (void)rcl_publish(&pose_publisher, &pose_message, nullptr);
}

}  // namespace

void microRosCommunicationTask(void* argument) {
    const auto context = *static_cast<CommunicationContext*>(argument);
    command_queue = context.command_queue;
    telemetry_queue = context.telemetry_queue;

    set_microros_serial_transports(Serial);
    vTaskDelay(pdMS_TO_TICKS(1500));

    uint32_t last_connection_attempt_ms = 0;
    uint32_t last_publish_ms = 0;
    uint32_t last_agent_ping_ms = 0;
    TelemetryFrame latest_telemetry{};

    for (;;) {
        const uint32_t now_ms = millis();

        if (!communication_ready) {
            if (now_ms - last_connection_attempt_ms >= robot_config::ros_agent_ping_period_ms) {
                last_connection_attempt_ms = now_ms;

                if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK && !createEntities()) {
                    destroyEntities();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (rclc_executor_spin_some(&executor,
                                    RCL_MS_TO_NS(robot_config::ros_spin_period_ms)) != RCL_RET_OK) {
            destroyEntities();
            continue;
        }

        while (telemetry_queue != nullptr &&
               xQueueReceive(telemetry_queue, &latest_telemetry, 0) == pdTRUE) {
        }

        if (now_ms - last_publish_ms >= robot_config::telemetry_period_ms) {
            last_publish_ms = now_ms;
            publishPose(latest_telemetry);
        }

        if (now_ms - last_agent_ping_ms >= robot_config::ros_agent_ping_period_ms) {
            last_agent_ping_ms = now_ms;
            if (rmw_uros_ping_agent(25, 1) != RMW_RET_OK) {
                destroyEntities();
                continue;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
