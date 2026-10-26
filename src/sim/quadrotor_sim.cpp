#include "sim/quadrotor_sim.hpp"

#include <algorithm>
#include <cmath>

namespace px4p::sim {

using math::Vector3f;

QuadrotorSim::QuadrotorSim(const QuadrotorParams& params)
    : params_(params), motor_positions_(motor_positions(params.arm_length_m)) {}

std::array<Vector3f, 4> QuadrotorSim::motor_positions(float arm_length_m) {
  const float d = arm_length_m / std::sqrt(2.f);
  return {{
      {+d, +d, 0.f},  // 1: front right
      {-d, -d, 0.f},  // 2: rear left
      {+d, -d, 0.f},  // 3: front left
      {-d, +d, 0.f},  // 4: rear right
  }};
}

float QuadrotorSim::hover_command() const {
  return params_.mass_kg * kGravity / (4.f * params_.max_thrust_per_motor_n);
}

void QuadrotorSim::step(const std::array<float, 4>& motor_commands, float dt) {
  // 1. Motors don't change speed instantly: first-order lag toward the command.
  const float alpha = dt / (params_.motor_time_constant_s + dt);
  for (size_t i = 0; i < 4; ++i) {
    const float target = std::clamp(motor_commands[i], 0.f, 1.f) * params_.max_thrust_per_motor_n;
    state_.motor_thrust_n[i] += (target - state_.motor_thrust_n[i]) * alpha;
  }

  // 2. Total thrust force and torque in the body frame. Each motor pushes along -z
  //    (up); torque is r x F plus the propeller's reaction torque about z.
  Vector3f force_body;
  Vector3f torque_body;
  for (size_t i = 0; i < 4; ++i) {
    const Vector3f thrust{0.f, 0.f, -state_.motor_thrust_n[i]};
    force_body += thrust;
    torque_body += motor_positions_[i].cross(thrust);
    torque_body.z += kSpinDirection[i] * params_.yaw_moment_coeff_m * state_.motor_thrust_n[i];
  }

  // Drag acts against the velocity, expressed in the body frame.
  const Vector3f velocity_body = state_.attitude.conjugate().rotate(state_.velocity_ned);
  force_body -= velocity_body.emult({params_.rotor_drag_n_per_m_s, params_.rotor_drag_n_per_m_s,
                                     params_.vertical_drag_n_per_m_s});

  // 3. Rotational dynamics, Euler's equation: I*w_dot = tau - w x (I*w).
  const Vector3f& inertia = params_.inertia_kg_m2;
  const Vector3f& w = state_.omega_body;
  const Vector3f w_dot = (torque_body - w.cross(w.emult(inertia))).edivide(inertia);

  // 4. Translational dynamics in NED.
  const Vector3f gravity{0.f, 0.f, kGravity};
  Vector3f accel_ned = state_.attitude.rotate(force_body) / params_.mass_kg + gravity;

  // Semi-implicit Euler: update velocities first, then positions with the new velocities.
  state_.omega_body += w_dot * dt;
  state_.velocity_ned += accel_ned * dt;
  state_.position_ned += state_.velocity_ned * dt;
  state_.attitude = state_.attitude.integrate(state_.omega_body, dt);

  // 5. Ground contact: the ground pushes back, so the vehicle rests instead of
  //    falling through. While resting it also can't rotate.
  if (state_.position_ned.z >= 0.f && accel_ned.z >= 0.f) {
    state_.position_ned.z = 0.f;
    state_.velocity_ned = {};
    state_.omega_body = {};
    accel_ned = {};
  }

  // Accelerometers measure specific force: everything except gravity.
  specific_force_body_ = state_.attitude.conjugate().rotate(accel_ned - gravity);
}

}  // namespace px4p::sim
