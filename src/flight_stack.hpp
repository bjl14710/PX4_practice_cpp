#pragma once

// FlightStack wires every module to one work queue and drives the lockstep
// loop: set the clock, run whatever is due, advance 1 ms, repeat.
//
// Data flow (each arrow is a uORB topic):
//
//   SimulatorBridge --sensor_imu_raw--> Sensors --sensor_imu--> AttitudeEstimator
//   AttitudeEstimator --vehicle_attitude--> McAttControl --vehicle_rates_setpoint-->
//   McRateControl --vehicle_torque_setpoint--> ControlAllocator --actuator_motors-->
//   SimulatorBridge (closes the loop through the physics)
//
// Topics and the clock are global (like a single PX4 instance), so only one
// FlightStack should exist at a time. Tests call uorb::reset_all() first.

#include "modules/attitude_estimator/attitude_estimator.hpp"
#include "modules/commander/commander.hpp"
#include "modules/control_allocator/control_allocator.hpp"
#include "modules/mc_att_control/mc_att_control.hpp"
#include "modules/mc_rate_control/mc_rate_control.hpp"
#include "modules/sensors/sensors.hpp"
#include "platform/work_queue.hpp"
#include "sim/quadrotor_sim.hpp"
#include "sim/simulator_bridge.hpp"

namespace px4p {

class FlightStack {
 public:
  static constexpr hrt_abstime kTickUs = 1000;  // lockstep step = fastest module interval

  explicit FlightStack(const sim::QuadrotorParams& vehicle = {});

  /// Adds an extra module (pilot, logger...). It runs after the built-in ones.
  void add(ScheduledWorkItem& item) { work_queue_.add(item); }

  /// Runs one tick: every due module runs, then the clock advances by kTickUs.
  void step();

  /// Steps until the clock reaches t_end_us.
  void run_until(hrt_abstime t_end_us);

  hrt_abstime now() const { return now_us_; }

  sim::QuadrotorSim& sim() { return sim_; }
  const Commander& commander() const { return commander_; }
  const AttitudeEstimator& estimator() const { return estimator_; }

 private:
  hrt_abstime now_us_{0};
  WorkQueue work_queue_;
  sim::QuadrotorSim sim_;

  // Declaration order = construction order = the order they run each tick.
  sim::SimulatorBridge bridge_;
  Sensors sensors_;
  AttitudeEstimator estimator_;
  Commander commander_;
  McAttControl att_control_;
  McRateControl rate_control_;
  ControlAllocator allocator_;
};

}  // namespace px4p
