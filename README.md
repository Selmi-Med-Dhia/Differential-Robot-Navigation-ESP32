# Differential Robot Navigation — ESP32

A compact differential-drive navigation stack for the classic ESP32. It keeps PCNT encoder counting, exact differential-drive odometry, wheel-speed feedback, straight-line correction, go-to positioning, rotations, emergency stop, and stall detection.

Bézier trajectories and the host simulation code have intentionally been removed to keep the project focused and easier to maintain.

## Choose the communication mode

There are two PlatformIO environments. Select the one you want before building or uploading:

- **`esp32dev_microros`** — communicates with ROS 2 through micro-ROS.
- **`esp32dev_uart`** — communicates through a simple line-based UART protocol on `Serial2`.

The selected backend only translates external messages into `NavCommand` objects and reads telemetry. The navigation/control task is identical in both builds.

### PlatformIO CLI

micro-ROS:

```bash
pio run -e esp32dev_microros
pio run -e esp32dev_microros -t upload
```

UART:

```bash
pio run -e esp32dev_uart
pio run -e esp32dev_uart -t upload
```

In the PlatformIO IDE you can select the corresponding environment from the project environments list before Build or Upload.

## Architecture

```text
                       core 0                         core 1
              +--------------------+        +------------------------+
External ---> | selected backend   | queue  | navigation_control     |
interface     | micro-ROS OR UART  |------->| PCNT encoder snapshot  |
              |                    |        | odometry               |
              | telemetry output   |<-------| navigation             |
              +--------------------+        | wheel controllers      |
                                            | motor PWM              |
                                            +------------------------+
```

Communication never owns the motor-control loop. A slow or disconnected host therefore cannot block encoder sampling or motor control.

## Navigation features

- full x4 quadrature decoding with ESP32 PCNT;
- exact SE(2) differential-drive odometry;
- independent right/left wheel speed controllers;
- static and velocity feed-forward;
- PI(D), filtered derivative, anti-windup, and PWM slew limiting;
- distance-based acceleration and braking;
- straight-line heading correction;
- Stanley-style cross-track correction;
- precise endpoint capture;
- forward and reverse movement;
- relative rotation;
- absolute orientation;
- go-to position with optional final orientation;
- velocity mode;
- emergency stop;
- wheel stall detection;
- 200 Hz control task pinned to ESP32 core 1.

## Hardware defaults

The defaults are in `include/robot_config.hpp`.

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
| Track width | 110.37343379 mm |
| Encoder CPR | 2800 counts/rev, x4 assumption |
| UART RX | GPIO 25 |
| UART TX | GPIO 26 |
| UART baud | 115200 |

Confirm the encoder CPR and signs on the physical robot before driving it.

## micro-ROS mode

The micro-ROS environment uses serial transport through the ESP32 `Serial` port.

Topics:

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | subscribe | linear x in m/s, angular z in rad/s |
| `/nav/goal` | `geometry_msgs/Pose2D` | subscribe | x/y in metres, theta in radians |
| `/nav/emergency_stop` | `std_msgs/Bool` | subscribe | true = emergency stop, false = clear |
| `/robot_pose_raw` | `geometry_msgs/Pose2D` | publish | odometry pose in metres/radians |

## UART mode

UART mode uses `Serial2`. Defaults are RX GPIO 25, TX GPIO 26, 115200 baud. Change them in `include/robot_config.hpp` if necessary.

Commands are ASCII lines terminated by `\n`. Spaces and commas are both accepted as separators.

```text
PING
STOP
ESTOP
CLEAR
VEL linear_mm_s angular_rad_s
MOVE distance_mm [speed_mm_s]
ROTATE angle_rad [wheel_speed_mm_s]
ORIENT angle_rad [wheel_speed_mm_s]
POSE x_mm y_mm theta_rad
GOTO x_mm y_mm speed_mm_s direction [final_heading_rad]
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

The ESP32 answers accepted commands with:

```text
OK
```

Errors look like:

```text
ERR,unknown command
```

On startup UART mode sends:

```text
READY,DIFFNAV_UART_V1
```

It also sends periodic telemetry:

```text
TEL,x_mm,y_mm,theta_rad,linear_mm_s,angular_rad_s,mode,result,fault,right_pwm,left_pwm
```

`mode`, `result`, and `fault` are the numeric values of the enums in `include/diffnav.hpp`.

## Calibration order

1. Verify both encoder signs by pushing the robot forward by hand.
2. Rotate each wheel one exact revolution and confirm x4 CPR.
3. Calibrate the two wheel diameters over a long measured straight distance.
4. Calibrate effective track width with several full rotations.
5. Characterize each motor's PWM dead zone/feed-forward.
6. Tune wheel PI gains.
7. Tune heading and cross-track gains only after odometry and wheel speed are trustworthy.

## Source layout

```text
include/
  communication.hpp   shared communication interface and telemetry
  diffnav.hpp         navigation, odometry, controller types
  encoder_pcnt.hpp    encoder driver interface
  motor_driver.hpp    motor driver interface
  robot_config.hpp    pins, calibration, timing, UART settings

src/
  diffnav.cpp             odometry and navigation implementation
  encoder_pcnt.cpp        full-quadrature PCNT implementation
  main.cpp                queues and deterministic control task
  micro_ros_bridge.cpp    micro-ROS backend
  uart_bridge.cpp         raw UART backend
  motor_driver.cpp        PWM/direction output
```
