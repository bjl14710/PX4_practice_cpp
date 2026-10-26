#include "modules/control_allocator/control_allocator.hpp"

#include <algorithm>

namespace px4p {

std::array<float, 4> mix(const math::Vector3f& torque, float thrust) {
  // Step 1: thrust + roll/pitch only.
  std::array<float, 4> out{};
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = thrust + kQuadXMixer[i].roll * torque.x + kQuadXMixer[i].pitch * torque.y;
  }

  // Step 2: fit roll/pitch into [0, 1]. If the spread between motors is
  // bigger than the whole range, shrink it; then shift every motor together.
  // Shifting changes thrust but keeps the roll/pitch differences intact.
  float lo = *std::min_element(out.begin(), out.end());
  float hi = *std::max_element(out.begin(), out.end());
  if (hi - lo > 1.f) {
    const float scale = 1.f / (hi - lo);
    for (float& u : out) {
      u = thrust + (u - thrust) * scale;
    }
    lo = *std::min_element(out.begin(), out.end());
    hi = *std::max_element(out.begin(), out.end());
  }
  if (hi > 1.f) {
    for (float& u : out) {
      u -= hi - 1.f;
    }
  } else if (lo < 0.f) {
    for (float& u : out) {
      u -= lo;
    }
  }

  // Step 3: yaw gets whatever headroom is left. Yaw authority on a quad is
  // weak and least important, so it is the first thing to give up.
  float yaw = torque.z;
  for (size_t i = 0; i < out.size(); ++i) {
    const float c = kQuadXMixer[i].yaw * yaw;  // this motor's yaw contribution
    if (c > 1e-6f && out[i] + c > 1.f) {
      yaw *= (1.f - out[i]) / c;
    } else if (c < -1e-6f && out[i] + c < 0.f) {
      yaw *= out[i] / -c;
    }
  }
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = std::clamp(out[i] + kQuadXMixer[i].yaw * yaw, 0.f, 1.f);
  }
  return out;
}

ControlAllocator::ControlAllocator() : ScheduledWorkItem("control_allocator", kIntervalUs) {}

void ControlAllocator::Run() {
  torque_sp_sub_.update(torque_sp_);
  status_sub_.update(status_);

  msg::ActuatorMotors motors{};
  motors.timestamp = hrt_absolute_time();
  if (status_.arming_state == msg::VehicleStatus::ArmingState::kArmed) {
    motors.control = mix(torque_sp_.torque, torque_sp_.thrust);
  }
  // Disarmed: control stays all zeros, motors stopped.
  motors_pub_.publish(motors);
}

}  // namespace px4p
