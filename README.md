# px4_practice_cpp

A small, standalone C++17 multirotor flight stack modelled on
[PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)'s architecture, flying a
simulated quadrotor in lockstep software-in-the-loop (SITL). The point is to
practice the ideas and C++ patterns you meet in real PX4 code, in a repo that
builds in seconds on any machine (laptop or remote session) with no PX4
toolchain, Gazebo, or hardware.

## Quick start

```bash
cmake -S . -B build -G Ninja        # or omit -G Ninja to use make
cmake --build build
ctest --test-dir build --output-on-failure
./build/px4p_sitl flight.csv        # fly the demo, write a CSV log
```

Requirements: a C++17 compiler (GCC 9+/Clang 10+), CMake 3.20+, and git
(GoogleTest is fetched automatically if it isn't installed).

## How it maps to PX4

| This repo | PX4 | Job |
|---|---|---|
| `src/uorb/` | `platforms/common/uORB` | publish/subscribe message bus |
| `src/msg/` | `msg/*.msg` | message definitions (topics) |
| `src/platform/work_queue` | `px4_work_queue`, `ScheduledWorkItem` | runs modules at fixed rates |
| `src/platform/hrt` | `drivers/drv_hrt.h` | microsecond clock (lockstep in SITL) |
| `modules/sensors` | `modules/sensors` | filter raw IMU |
| `modules/attitude_estimator` | `attitude_estimator_q` (pre-EKF2) | gyro + accel complementary filter |
| `modules/mc_att_control` | `modules/mc_att_control` | attitude error → body rates (P) |
| `modules/mc_rate_control` | `modules/mc_rate_control` | rate error → torque (PID) |
| `modules/control_allocator` | `modules/control_allocator` / mixers | torque + thrust → motors |
| `modules/commander` | `modules/commander` | arming state machine, preflight checks |
| `modules/logger` | `modules/logger` (ULog) | CSV log |
| `sim/` | Gazebo / jMAVSim + `simulator_mavlink` | quadrotor physics + IMU |

Data flow (every arrow is a uORB topic):

```
SimulatorBridge ─sensor_imu_raw─▶ Sensors ─sensor_imu─▶ AttitudeEstimator ─vehicle_attitude─┐
      ▲                                                                                     ▼
      └─actuator_motors─ ControlAllocator ◀─torque_sp─ McRateControl ◀─rates_sp─ McAttControl
                                                                                     ▲
                       FlightScript (scripted pilot) ─vehicle_attitude_setpoint──────┘
```

Conventions follow PX4: world frame **NED** (z down, altitude = −z), body frame
**FRD** (x forward, y right, z down), quaternions are Hamilton `(w, x, y, z)`
rotating body → world, motors use PX4's Quadrotor X numbering.

## Layout

```
src/
  platform/   clock and work queue
  uorb/       Topic<T>, Publication<T>, Subscription<T>
  msg/        message structs
  lib/mathlib Vector3f, Quatf, Pid, LowPassFilter
  modules/    one folder per module, like PX4
  sim/        physics + simulator bridge
  flight_stack.*  wires everything together; main.cpp runs the demo
tests/        GoogleTest unit tests + closed-loop SITL tests
docs/
  EXERCISES.md  practice tasks, easiest first
  learning/     /teach lessons, /drill-me drills, PROGRESS.md, GLOSSARY.md
.claude/      Claude Code commands (/teach, /drill-me, /grill-me, ...)
```

## Practising

- **Exercises:** [`docs/EXERCISES.md`](docs/EXERCISES.md): altitude hold,
  a magnetometer, failsafes, a parameter system and more.
- **Lessons:** run `/teach <concept>` in Claude Code. Lessons are HTML files in
  `docs/learning/lessons/`, built from this repo's own code. `/drill-me` creates
  self-tests in `docs/learning/drills/`. Progress is tracked in
  `docs/learning/PROGRESS.md`, so it carries over between sessions on any machine.

## Known limitations (on purpose)

- **No altitude or position control.** Altitude is flown open loop; adding it
  is exercise 1.
- **Yaw drifts.** There's no magnetometer, so yaw is gyro-only.
- **Tilt estimate lags during sustained tilts.** A multirotor's accelerometer
  mostly measures thrust, not gravity, so the complementary filter is pulled
  toward "level" while the vehicle is tilted. PX4 solves this with GPS velocity
  aiding (EKF2). It's a great thing to study.
- Single process, single thread, one vehicle; topics have a queue depth of 1.
