# Differential Robot Navigation — ESP32

High-accuracy differential-drive navigation for a classic ESP32, distilled from the Aerobotix STM32/micro-ROS navigation stack and adapted to ESP32 hardware constraints. The design keeps the useful mathematics and control structure while removing STM32 HAL/timer dependencies, blocking trajectory loops, and expensive repeated curve calculations.

## What is preserved / improved

- **Full x4 quadrature PCNT encoders** — no GPIO encoder ISR path and no STM32 encoder-timer mode.
- **64-bit monotonic encoder totals** on the current Arduino-ESP32/IDF4 path; the IDF5+ path uses PCNT hardware accumulation/watch points.
- **Exact SE(2) differential-drive odometry** for each wheel increment instead of a first-order pose approximation.
- **Independent wheel-speed feedback** with static/velocity feed-forward, PI(D), derivative-on-measurement filtering, anti-windup, and PWM slew limiting.
- **Distance-domain acceleration and braking** using `v² = v0² + 2as`. This retains the STM32 project's distance-based profiling idea with simpler and dimensionally consistent math.
- **Straight-line correction** using heading feedback plus Stanley-style cross-track correction.
- **Precise point capture** in the last 45 mm, followed by heading settling. This avoids the lateral deadlock possible when only along-track error is controlled.
- **Cubic Bézier following** with:
  - a 64-sample arc-length LUT built once per command;
  - local projection + four Newton iterations per control step;
  - signed curvature feed-forward;
  - heading + cross-track feedback;
  - a robust endpoint handoff to the point-capture controller.
- **Forward and reverse trajectories**, relative/absolute rotation, go-to, velocity mode, emergency stop, and stall detection.
- **Non-blocking state machine**. Navigation never suspends the micro-ROS task and micro-ROS never owns the motor loop.
- **200 Hz control task pinned to core 1**, with micro-ROS on core 0.

## Architecture

```text
                     core 0                                  core 1
        +-----------------------------+        +--------------------------------+
ROS 2 ->| micro-ROS subscriptions      | queue  | 200 Hz navigation_control task |
        | /cmd_vel                    |------->|  PCNT snapshot                 |
        | /nav/goal                   |        |  exact odometry                |
        | /nav/emergency_stop         |        |  trajectory controller         |
        |                             |        |  wheel speed controllers       |
        | /robot_pose_raw publisher   |<-------|  PWM motor output              |
        +-----------------------------+ telem. +--------------------------------+
```

The code under `include/diffnav/` and `src/core/` has no Arduino dependency. The exact same navigation logic is therefore compiled and exercised by the host simulation.

## Hardware defaults

The initial pinout/calibration comes from the working `PAMI_ESP32_code` robot and lives only in `include/robot_config.hpp`:

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
| Encoder CPR | 2800 counts/rev (x4 assumption) |

### Important CPR note

The PAMI firmware configured `CPR=1400` while counting only rising + falling edges of one encoder phase (x2). This project counts both phases (x4), so the equivalent default is **2800 counts/revolution**. Confirm this on the physical robot: rotate one wheel exactly one revolution and read the PCNT delta. If it is not 2800, change `kCountsPerRevolutionX4`.

Also verify the sign of each wheel by pushing the robot forward by hand. Both software counts must increase. If one decreases, flip its `k...EncoderSign` in `robot_config.hpp` rather than swapping random control equations.

## Build

The project uses PlatformIO and the maintained `micro_ros_platformio` integration.

```bash
pio run
pio run -t upload
```

Serial micro-ROS transport is the default, so no Wi-Fi credentials are compiled into the firmware. Run an agent on the host, for example:

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged \
  microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6
```

### ROS topics

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | subscribe | linear x in m/s, angular z in rad/s |
| `/nav/goal` | `geometry_msgs/Pose2D` | subscribe | x/y in metres, theta in radians; forward go-to + final heading |
| `/nav/emergency_stop` | `std_msgs/Bool` | subscribe | `true` latches stop, `false` clears it |
| `/robot_pose_raw` | `geometry_msgs/Pose2D` | publish | odometry pose in metres/radians |

The core also supports reverse go-to, distance moves, relative/absolute rotations, and cubic Bézier commands through `diffnav::Navigator`. If the team wants the exact custom STM32 `NaviStm` service API, add that generated interface under `extra_packages/` and translate it into the existing `NavCommand` queue; do **not** put ROS calls into the 200 Hz control task.

## Host simulation

Run:

```bash
g++ -O2 -std=c++17 -Wall -Wextra -Wpedantic -Werror -Iinclude \
  sim/sim_main.cpp src/core/odometry.cpp src/core/bezier.cpp src/core/navigator.cpp \
  -o sim/sim
./sim/sim
```

The simulator includes legacy-PCNT limit-reset reconstruction tests, encoder quantization, a 55 ms motor time constant, PWM dead-zone/slope mismatch between left and right motors, and the actual wheel-speed controller. Current deterministic results:

| Scenario | Result | Position error | Heading error |
|---|---|---:|---:|
| PCNT limit extension | PASS | — | — |
| Exact odometry arc | PASS | 0.000 mm | 0.000° |
| Forward 1000 mm | PASS | 1.829 mm | 0.766° |
| Reverse 600 mm | PASS | 0.504 mm | 0.289° |
| Rotate 90° | PASS | 2.112 mm center drift | 0.113° |
| Go-to (700, 500) + final heading | PASS | 3.421 mm | 0.136° |
| Forward cubic Bézier | PASS | 0.658 mm | 0.649° |
| Reverse cubic Bézier | PASS | 1.674 mm | 0.098° |

These are **software/model validation results, not a claim of real-world millimetre accuracy**. Real accuracy will depend mainly on wheel-diameter calibration, effective track width, encoder CPR, tire slip, gearbox backlash, motor feed-forward, and chassis flex.

## Calibration order

1. **Encoder signs and x4 CPR** — verify by hand before enabling motors.
2. **Wheel diameters** — drive a long straight measured distance and calibrate left/right independently.
3. **Track width** — command several full rotations and tune until accumulated heading is correct.
4. **PWM dead-zone + feed-forward slope** — characterize each wheel with the robot lifted, then under normal floor load.
5. **Wheel PI gains** — tune speed tracking before touching path gains.
6. **Path gains** — tune heading/cross-track correction only after odometry and wheel speed are trustworthy.

## Why this is lighter than the STM32 project

The ESP32 version intentionally does not port STM32 peripheral HAL, encoder timer overflow code, repeated Bézier length integration, blocking motion `while` loops, or direct-PWM final-position loops. Instead, expensive curve quantities are precomputed, all motion shares the same wheel-speed control layer, and communication is isolated from the real-time task.

That keeps most of the navigation capability while giving the ESP32 a bounded amount of work every 5 ms.
