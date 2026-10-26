#pragma once

// Connects the physics to the flight stack, like PX4's simulator_mavlink:
// it consumes actuator_motors and publishes noisy IMU samples.

#include <cstdint>
#include <random>

#include "msg/actuator_motors.hpp"
#include "msg/sensor_imu_raw.hpp"
#include "platform/work_queue.hpp"
#include "sim/quadrotor_sim.hpp"
#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p::sim {

struct SensorNoise {
  float accel_stddev_m_s2{0.05f};
  float gyro_stddev_rad_s{0.003f};
};

class SimulatorBridge final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 1000;  // 1 kHz physics + IMU

  SimulatorBridge(QuadrotorSim& sim, const SensorNoise& noise = {}, uint32_t seed = 1);

  void Run() override;

 private:
  QuadrotorSim& sim_;
  uorb::Subscription<msg::ActuatorMotors> motors_sub_;
  uorb::Publication<msg::SensorImuRaw> imu_pub_;
  msg::ActuatorMotors motors_{};

  // Fixed seed => the same noise every run, so tests are repeatable.
  std::mt19937 rng_;
  std::normal_distribution<float> accel_noise_;
  std::normal_distribution<float> gyro_noise_;
};

}  // namespace px4p::sim
