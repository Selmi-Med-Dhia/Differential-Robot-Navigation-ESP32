# Differential Robot Navigation — ESP32

Differential-drive navigation for ESP32 using PCNT encoders, exact odometry, wheel-speed feedback, distance-based acceleration/braking, lane correction, go-to positioning, rotations, emergency stop, and stall detection.

Bézier navigation and the old host simulation code are intentionally not part of this project.

## Communication selection

There is now **one PlatformIO build**. Communication is selected in code, in `include/robot_config.hpp`:

```cpp
enum class CommunicationType : uint8_t {
    MICRO_ROS,
    UART,
};

constexpr CommunicationType communication_type = CommunicationType::MICRO_ROS;
```

To use UART, change only the last line to:

```cpp
constexpr CommunicationType communication_type = CommunicationType::UART;
```

Then build/upload normally:

```bash
pio run
pio run -t upload
```

Both communication backends are compiled in the same project. `setup()` starts only the task selected by `communication_type`, while the navigation/control task is identical in both cases.

## Naming

The code deliberately follows the vocabulary used in the supplied STM32/PAMI navigation code where practical:

- `wheel_spacing_mm`, not `track_width`;
- `wheel_diameter_R_mm` / `wheel_diameter_L_mm`;
- `encoder_count_R` / `encoder_count_L`;
- `speed_R_mm_s` / `speed_L_mm_s`;
- `phi_rad` and `absolute_phi_rad`;
- `ramping_acceleration`, `breaking_acceleration`, `rotating_speed`;
- `lane_error` and lane correction terminology;
- navigation methods such as `moveForward`, `rotate`, `orientate`, and `goTo`.

The command structure also uses named values (`x_mm`, `y_mm`, `phi_rad`, `distance_mm`, `speed_mm_s`) instead of opaque `p0/p1/p2` fields.

## Driven-wheel / encoder-wheel speed convention

This project intentionally keeps the same assumption as the supplied STM32 navigation code.

The encoders are on the outer/dead wheels. During a turn, the physical linear speed of an outer encoder wheel is not exactly the same as the linear speed of the corresponding driven wheel because the two wheels are at different distances from the robot rotation center.

A more geometric model could use robot angular velocity and the two different wheel spacings to convert driven-wheel velocity into encoder-wheel velocity. **This project intentionally does not do that conversion.**

The targets are used 1:1:

```text
target_encoder_speed_R = driven_wheel_speed_R
target_encoder_speed_L = driven_wheel_speed_L
```

The wheel PID therefore compares the driven-wheel target speed directly with the measured outer/dead-wheel encoder speed, exactly like `set_target_speeds()` in the STM32 implementation. Do not add an angular-velocity compensation term between these two values unless the control strategy is intentionally being changed.

## Architecture

```text
                       core 0                         core 1
              +--------------------+        +------------------------+
External ---> | communication task | queue  | navigation_control     |
interface     | micro-ROS OR UART  |------->| PCNT encoder snapshot  |
              | selected in code   |        | odometry               |
              | telemetry output   |<-------| navigation             |
              +--------------------+        | wheel speed PID        |
                                            | motor PWM              |
                                            +------------------------+
```

Communication never owns the motor-control loop. A disconnected ROS agent or a slow UART peer therefore cannot block encoder sampling or motor control.

## Navigation features

- full x4 quadrature decoding with ESP32 PCNT;
- monotonic software-extended encoder count on the legacy ESP32 PCNT driver;
- exact circular-arc differential-drive odometry;
- independent right/left wheel speed controllers;
- driven-wheel targets used directly as encoder-wheel PID targets, matching the STM32 convention;
- static + velocity feed-forward;
- PI(D), filtered derivative, anti-windup, and PWM slew limiting;
- distance-domain ramping and braking;
- straight movement with phi correction and lane correction;
- precise final-point capture;
- forward and backwards movement;
- relative rotation;
- absolute orientation;
- go-to position with optional final phi;
- direct velocity mode;
- emergency break and wheel stall detection;
- 200 Hz control task pinned to ESP32 core 1.

## Hardware defaults

All values are in `include/robot_config.hpp`.

| Parameter | Default |
|---|---:|
| Right motor direction | GPIO 19 / 18 |
| Left motor direction | GPIO 21 / 22 |
| Right PWM enable | GPIO 17 |
| Left PWM enable | GPIO 23 |
| PWM | 20 kHz, 10 bit |
| Right encoder A/B | GPIO 32 / 15 |
| Left encoder A/B | GPIO 16 / 4 |
| Right wheel diameter | 34.73184056 mm |
| Left wheel diameter | 34.69274232 mm |
| Wheel spacing | 110.37343379 mm |
| Encoder CPR | 2800 counts/rev, x4 assumption |
| UART RX | GPIO 25 |
| UART TX | GPIO 26 |
| UART baud | 115200 |

Confirm encoder directions, CPR, wheel diameters, and wheel spacing on the physical robot before relying on position accuracy.

## micro-ROS

micro-ROS uses serial transport through ESP32 `Serial`.

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | subscribe | linear x in m/s, angular z in rad/s |
| `/nav/goal` | `geometry_msgs/Pose2D` | subscribe | x/y in metres, theta mapped to robot phi |
| `/nav/emergency_stop` | `std_msgs/Bool` | subscribe | true = emergency break, false = clear |
| `/robot_pose_raw` | `geometry_msgs/Pose2D` | publish | x/y in metres and robot phi |

## UART

UART uses `Serial2` with RX GPIO 25, TX GPIO 26 and 115200 baud by default.

Commands are ASCII lines terminated by `\n`. Spaces and commas are accepted as separators.

```text
PING
STOP
ESTOP
CLEAR
VEL speed_mm_s angular_speed_rad_s
MOVE distance_mm [cruise_speed_mm_s]
ROTATE angle_rad [rotating_speed_mm_s]
ORIENT phi_rad [rotating_speed_mm_s]
POSE x_mm y_mm phi_rad
GOTO x_mm y_mm speed_mm_s direction [final_phi_rad]
```

Examples:

```text
MOVE 1000 400
ROTATE 1.570796 220
GOTO 700 500 350 1 1.570796
GOTO 300 200 250 -1
VEL 200 0.5
STOP
```

Accepted commands return `OK`; malformed commands return `ERR,<reason>`. UART telemetry is:

```text
TEL,x_mm,y_mm,phi_rad,speed_mm_s,angular_speed_rad_s,mode,result,fault,pwm_R,pwm_L
```

## Calibration order

1. Push the robot forward by hand and confirm both `encoder_count_R` and `encoder_count_L` increase.
2. Rotate each wheel exactly one revolution and verify `encoder_counts_per_revolution`.
3. Calibrate `wheel_diameter_R_mm` and `wheel_diameter_L_mm` over a long straight distance.
4. Calibrate `wheel_spacing_mm` using several complete rotations.
5. Characterize motor dead-zone/feed-forward.
6. Tune wheel speed PI gains.
7. Tune phi correction and lane correction only after odometry and wheel-speed control are trustworthy.

## Source layout

```text
include/
  communication.hpp   shared queues/telemetry and both communication task declarations
  diffnav.hpp         navigation, odometry, speed control types
  encoder_pcnt.hpp    ESP32 PCNT encoder interface
  motor_driver.hpp    motor PWM/direction interface
  robot_config.hpp    communication selector, pins, calibration and gains

src/
  diffnav.cpp             odometry and navigation logic
  encoder_pcnt.cpp        full x4 PCNT implementation
  main.cpp                command dispatcher and deterministic control task
  micro_ros_bridge.cpp    micro-ROS backend
  uart_bridge.cpp         UART backend
  motor_driver.cpp        motor output
```
