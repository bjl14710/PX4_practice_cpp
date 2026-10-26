# Exercises

Practice tasks, easiest first. Each one adds a real PX4 concept. Suggested
workflow per exercise: `/grill-me` your plan, implement with tests, run
`ctest`, then `/teach` or `/drill-me` the concept it introduced.

Run the stack after every change: `./build/px4p_sitl flight.csv`, then look at
the CSV (`roll_deg` vs `true_roll_deg` and so on).

## 1. Barometer + altitude hold
*Concepts: new sensor, new topic, cascaded control.*
- Add a simulated barometer to `SimulatorBridge` (altitude + noise, ~50 Hz) on a
  new `sensor_baro` topic.
- Add an `altitude_estimator` (a complementary filter of baro and accel z) that
  publishes `vehicle_local_position` (z and vz).
- Add an `mc_pos_control`-style altitude controller: altitude error → climb rate
  (P) → thrust (PI around hover thrust).
- Change `demo_flight` to command altitudes instead of raw thrust.
- Test: hold 2 m within ±0.2 m for 5 s in a closed-loop test.

## 2. Failsafe: IMU timeout
*Concepts: state machines, health monitoring.*
- In `Commander`, detect an IMU that is stale while armed.
- Don't disarm in the air! Add a `kFailsafe` state that commands level attitude
  and slowly reduces thrust (a "descend" failsafe).
- Test: stop the IMU mid-flight (add a switch to `SimulatorBridge`) and check the
  state changes and the vehicle descends.

## 3. Parameters
*Concepts: PX4 param system, compile-time vs run-time configuration.*
- Make a tiny `param` registry: name → float, with defaults, `param_get` /
  `param_set`, and a `parameter_update` topic published on change.
- Make `McRateControl` reload its gains when `parameter_update` arrives (PX4's
  `updateParams()` pattern).
- Test: change `MC_ROLLRATE_P` mid-flight and check the controller uses it.

## 4. Magnetometer + yaw correction
*Concepts: sensor fusion, observability.*
- Simulate a magnetometer (earth field rotated into the body frame + noise).
- In `AttitudeEstimator`, correct yaw using the horizontal part of the mag
  vector (only yaw, so mag errors don't disturb roll and pitch).
- Test: with a constant gyro z bias, yaw no longer drifts.

## 5. Fix the tilt-estimate lag
*Concepts: why accelerometers lie on multirotors, aiding.*
- Reproduce it: test that a 10° roll held for 3 s gives a large estimate error.
- Fix A: lower the accel weight while thrust is far from hover.
- Fix B (PX4-style): with a velocity estimate (from exercise 1 + GPS), subtract
  the vehicle's acceleration from the accel reading before using it as "down".
- Compare both in the CSV.

## 6. Thread-safe multi-rate queue
*Concepts: std::thread, condition variables, real-time scheduling.*
- Run the sim and the controllers on two work queue threads (PX4 has
  `rate_ctrl`, `nav_and_controllers`, ...). Use real time instead of lockstep.
- Notice what breaks (determinism) and what the uORB mutex now protects.

## 7. Port a module into real PX4
*Concepts: the real thing.*
- Clone PX4-Autopilot, build `make px4_sitl gz_x500`.
- Re-write `flight_script` as a PX4 external module that publishes
  `vehicle_attitude_setpoint` in offboard mode, and fly it in Gazebo.
