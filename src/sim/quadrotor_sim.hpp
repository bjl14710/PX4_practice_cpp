#pragma once

// Rigid-body quadrotor physics: the stand-in for Gazebo / jMAVSim.
//
// Frames: world is NED (z down, so altitude = -position.z), body is FRD.
// Motors use PX4's Quadrotor X numbering:
//
//                  front (+x)
//        3 (CW)  o     ^     o  1 (CCW)
//                      |
//                      +-----> right (+y)
//
//        2 (CCW) o           o  4 (CW)
//                    rear

#include <array>

#include "lib/mathlib/constants.hpp"
#include "lib/mathlib/quaternion.hpp"
#include "lib/mathlib/vector3.hpp"

namespace px4p::sim {

using math::kGravity;

struct QuadrotorParams {
  float mass_kg{1.0f};
  math::Vector3f inertia_kg_m2{0.01f, 0.01f, 0.018f};  ///< diagonal of the inertia tensor
  float arm_length_m{0.15f};          ///< center to motor
  float max_thrust_per_motor_n{4.0f};
  float yaw_moment_coeff_m{0.02f};    ///< reaction torque per newton of thrust
  float motor_time_constant_s{0.02f}; ///< first-order spin-up lag
  /// Rotor drag: spinning props resist sideways airflow in the rotor plane
  /// (body x/y). It is why a hovering multirotor's accelerometer still "sees"
  /// a tilt once the vehicle reaches a steady velocity.
  float rotor_drag_n_per_m_s{0.35f};
  float vertical_drag_n_per_m_s{0.1f};  ///< along body z
};

struct QuadrotorState {
  math::Vector3f position_ned;   ///< [m]
  math::Vector3f velocity_ned;   ///< [m/s]
  math::Quatf attitude;          ///< body -> world
  math::Vector3f omega_body;     ///< [rad/s]
  std::array<float, 4> motor_thrust_n{};
};

class QuadrotorSim {
 public:
  explicit QuadrotorSim(const QuadrotorParams& params = {});

  /// Advances the physics by dt seconds with normalized motor commands [0, 1].
  void step(const std::array<float, 4>& motor_commands, float dt);

  const QuadrotorState& state() const { return state_; }
  void set_state(const QuadrotorState& state) { state_ = state; }
  const QuadrotorParams& params() const { return params_; }

  /// What an ideal accelerometer reads: (acceleration - gravity) in body frame.
  const math::Vector3f& specific_force_body() const { return specific_force_body_; }

  /// Motor command that exactly cancels gravity when level.
  float hover_command() const;

  /// Height above ground [m] (NED z is down).
  float altitude_m() const { return -state_.position_ned.z; }

  bool on_ground() const { return state_.position_ned.z >= 0.f; }

  /// Motor positions in the body frame and spin directions (+1 CCW, -1 CW).
  static std::array<math::Vector3f, 4> motor_positions(float arm_length_m);
  static constexpr std::array<float, 4> kSpinDirection{+1.f, +1.f, -1.f, -1.f};

 private:
  QuadrotorParams params_;
  QuadrotorState state_;
  std::array<math::Vector3f, 4> motor_positions_;
  math::Vector3f specific_force_body_{0.f, 0.f, -kGravity};
};

}  // namespace px4p::sim
