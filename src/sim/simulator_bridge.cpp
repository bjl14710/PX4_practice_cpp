#include "sim/simulator_bridge.hpp"

namespace px4p::sim {

SimulatorBridge::SimulatorBridge(QuadrotorSim& sim, const SensorNoise& noise, uint32_t seed)
    : ScheduledWorkItem("simulator_bridge", kIntervalUs),
      sim_(sim),
      rng_(seed),
      accel_noise_(0.f, noise.accel_stddev_m_s2),
      gyro_noise_(0.f, noise.gyro_stddev_rad_s) {}

void SimulatorBridge::Run() {
  // Keep the last command if nothing new arrived (motors hold their value).
  motors_sub_.update(motors_);

  sim_.step(motors_.control, us_to_s(interval_us()));

  const math::Vector3f& accel = sim_.specific_force_body();
  const math::Vector3f& gyro = sim_.state().omega_body;

  msg::SensorImuRaw imu{};
  imu.timestamp = hrt_absolute_time();
  imu.accel_m_s2 = {accel.x + accel_noise_(rng_), accel.y + accel_noise_(rng_),
                    accel.z + accel_noise_(rng_)};
  imu.gyro_rad_s = {gyro.x + gyro_noise_(rng_), gyro.y + gyro_noise_(rng_),
                    gyro.z + gyro_noise_(rng_)};
  imu_pub_.publish(imu);
}

}  // namespace px4p::sim
