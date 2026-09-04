#include "communication.hpp"

#if defined(COMMUNICATION_UART)

#include <cstdlib>
#include <cstring>

#include "robot_config.hpp"

namespace {

HardwareSerial navigation_uart(2);
QueueHandle_t command_queue = nullptr;
QueueHandle_t telemetry_queue = nullptr;

constexpr size_t kCommandBufferSize = 160;
char command_buffer[kCommandBufferSize] = {};
size_t command_length = 0;

void enqueueCommand(const diffnav::NavCommand& command) {
    if (command_queue == nullptr) {
        return;
    }

    if (xQueueSend(command_queue, &command, 0) == pdTRUE) {
        return;
    }

    diffnav::NavCommand discarded;
    xQueueReceive(command_queue, &discarded, 0);
    xQueueSend(command_queue, &command, 0);
}

bool parseFloat(const char* text, float& value) {
    if (text == nullptr) {
        return false;
    }

    char* end = nullptr;
    value = std::strtof(text, &end);
    return end != text && *end == '\0';
}

bool parseInt(const char* text, int& value) {
    if (text == nullptr) {
        return false;
    }

    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return false;
    }

    value = static_cast<int>(parsed);
    return true;
}

void writeOk() {
    navigation_uart.println("OK");
}

void writeError(const char* message) {
    navigation_uart.print("ERR,");
    navigation_uart.println(message);
}

void handleCommand(char* line) {
    char* save = nullptr;
    const char* name = strtok_r(line, " ,\t", &save);

    if (name == nullptr) {
        return;
    }

    diffnav::NavCommand command;

    if (strcmp(name, "STOP") == 0) {
        command.type = diffnav::NavCommandType::STOP;
        enqueueCommand(command);
        writeOk();
        return;
    }

    if (strcmp(name, "ESTOP") == 0) {
        command.type = diffnav::NavCommandType::EMERGENCY_STOP;
        enqueueCommand(command);
        writeOk();
        return;
    }

    if (strcmp(name, "CLEAR") == 0) {
        command.type = diffnav::NavCommandType::CLEAR_EMERGENCY;
        enqueueCommand(command);
        writeOk();
        return;
    }

    if (strcmp(name, "PING") == 0) {
        navigation_uart.println("PONG");
        return;
    }

    if (strcmp(name, "VEL") == 0) {
        if (!parseFloat(strtok_r(nullptr, " ,\t", &save), command.p0) ||
            !parseFloat(strtok_r(nullptr, " ,\t", &save), command.p1)) {
            writeError("VEL expects: VEL linear_mm_s angular_rad_s");
            return;
        }

        command.type = diffnav::NavCommandType::VELOCITY;
        enqueueCommand(command);
        writeOk();
        return;
    }

    if (strcmp(name, "MOVE") == 0) {
        if (!parseFloat(strtok_r(nullptr, " ,\t", &save), command.p0)) {
            writeError("MOVE expects: MOVE distance_mm [speed_mm_s]");
            return;
        }

        const char* speed_text = strtok_r(nullptr, " ,\t", &save);
        if (speed_text != nullptr && !parseFloat(speed_text, command.p1)) {
            writeError("invalid MOVE speed");
            return;
        }

        command.type = diffnav::NavCommandType::MOVE_DISTANCE;
        enqueueCommand(command);
        writeOk();
        return;
    }

    if (strcmp(name, "ROTATE") == 0) {
        if (!parseFloat(strtok_r(nullptr, " ,\t", &save), command.p0)) {
            writeError("ROTATE expects: ROTATE angle_rad [wheel_speed_mm_s]");
            return;
        }

        const char* speed_text = strtok_r(nullptr, " ,\t", &save);
        if (speed_text != nullptr && !parseFloat(speed_text, command.p1)) {
            writeError("invalid ROTATE speed");
            return;
        }

        command.type = diffnav::NavCommandType::ROTATE_RELATIVE;
        enqueueCommand(command);
        writeOk();
        return;
    }

    if (strcmp(name, "ORIENT") == 0) {
        if (!parseFloat(strtok_r(nullptr, " ,\t", &save), command.p0)) {
            writeError("ORIENT expects: ORIENT angle_rad [wheel_speed_mm_s]");
            return;
        }

        const char* speed_text = strtok_r(nullptr, " ,\t", &save);
        if (speed_text != nullptr && !parseFloat(speed_text, command.p1)) {
            writeError("invalid ORIENT speed");
            return;
        }

        command.type = diffnav::NavCommandType::ORIENT_ABSOLUTE;
        enqueueCommand(command);
        writeOk();
        return;
    }

    if (strcmp(name, "POSE") == 0) {
        if (!parseFloat(strtok_r(nullptr, " ,\t", &save), command.p0) ||
            !parseFloat(strtok_r(nullptr, " ,\t", &save), command.p1) ||
            !parseFloat(strtok_r(nullptr, " ,\t", &save), command.p2)) {
            writeError("POSE expects: POSE x_mm y_mm theta_rad");
            return;
        }

        command.type = diffnav::NavCommandType::SET_POSE;
        enqueueCommand(command);
        writeOk();
        return;
    }

    if (strcmp(name, "GOTO") == 0) {
        int direction = 1;

        if (!parseFloat(strtok_r(nullptr, " ,\t", &save), command.p0) ||
            !parseFloat(strtok_r(nullptr, " ,\t", &save), command.p1) ||
            !parseFloat(strtok_r(nullptr, " ,\t", &save), command.p2) ||
            !parseInt(strtok_r(nullptr, " ,\t", &save), direction)) {
            writeError("GOTO expects: GOTO x_mm y_mm speed_mm_s direction [final_heading_rad]");
            return;
        }

        command.direction = direction >= 0 ? 1 : -1;

        const char* final_heading_text = strtok_r(nullptr, " ,\t", &save);
        if (final_heading_text != nullptr) {
            if (!parseFloat(final_heading_text, command.p3)) {
                writeError("invalid GOTO final heading");
                return;
            }
            command.use_final_heading = true;
        }

        command.type = diffnav::NavCommandType::GO_TO;
        enqueueCommand(command);
        writeOk();
        return;
    }

    writeError("unknown command");
}

void readIncomingBytes() {
    while (navigation_uart.available() > 0) {
        const char byte = static_cast<char>(navigation_uart.read());

        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            command_buffer[command_length] = '\0';
            handleCommand(command_buffer);
            command_length = 0;
            continue;
        }

        if (command_length + 1 < kCommandBufferSize) {
            command_buffer[command_length++] = byte;
        } else {
            command_length = 0;
            writeError("command too long");
        }
    }
}

void writeTelemetry(const TelemetryFrame& frame) {
    navigation_uart.print("TEL,");
    navigation_uart.print(frame.odometry.pose.x_mm, 3);
    navigation_uart.print(',');
    navigation_uart.print(frame.odometry.pose.y_mm, 3);
    navigation_uart.print(',');
    navigation_uart.print(frame.odometry.pose.theta_rad, 6);
    navigation_uart.print(',');
    navigation_uart.print(frame.odometry.body_velocity.linear_mm_s, 3);
    navigation_uart.print(',');
    navigation_uart.print(frame.odometry.body_velocity.angular_rad_s, 6);
    navigation_uart.print(',');
    navigation_uart.print(static_cast<int>(frame.motion.mode));
    navigation_uart.print(',');
    navigation_uart.print(static_cast<int>(frame.motion.result));
    navigation_uart.print(',');
    navigation_uart.print(static_cast<int>(frame.motion.fault));
    navigation_uart.print(',');
    navigation_uart.print(frame.right_pwm, 1);
    navigation_uart.print(',');
    navigation_uart.println(frame.left_pwm, 1);
}

}  // namespace

void communicationTask(void* argument) {
    const auto context = *static_cast<CommunicationContext*>(argument);
    command_queue = context.command_queue;
    telemetry_queue = context.telemetry_queue;

    navigation_uart.begin(robot_config::kUartBaudRate,
                          SERIAL_8N1,
                          robot_config::kUartRxPin,
                          robot_config::kUartTxPin);

    navigation_uart.println("READY,DIFFNAV_UART_V1");

    TelemetryFrame latest_telemetry{};
    uint32_t last_telemetry_ms = 0;

    for (;;) {
        readIncomingBytes();

        while (telemetry_queue != nullptr &&
               xQueueReceive(telemetry_queue, &latest_telemetry, 0) == pdTRUE) {
        }

        const uint32_t now_ms = millis();
        if (now_ms - last_telemetry_ms >= robot_config::kUartTelemetryPeriodMs) {
            last_telemetry_ms = now_ms;
            writeTelemetry(latest_telemetry);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

#endif
