#pragma once

// Unit quaternion for attitude, modelled on PX4's matrix::Quatf.
//
// Conventions (same as PX4):
//  - Hamilton product, stored as (w, x, y, z).
//  - A vehicle attitude q rotates vectors FROM the body frame (FRD:
//    x forward, y right, z down) TO the world frame (NED: north, east, down).
//  - Euler angles are ZYX: yaw about z, then pitch about y, then roll about x.

#include <algorithm>
#include <cmath>

#include "lib/mathlib/vector3.hpp"

namespace px4p::math {

struct Quatf {
  float w{1.f};
  float x{0.f};
  float y{0.f};
  float z{0.f};

  constexpr Quatf() = default;
  constexpr Quatf(float w_in, float x_in, float y_in, float z_in)
      : w(w_in), x(x_in), y(y_in), z(z_in) {}

  /// Rotation of `angle_rad` about `axis` (axis need not be unit length).
  static Quatf from_axis_angle(const Vector3f& axis, float angle_rad) {
    const Vector3f u = axis.normalized();
    const float s = std::sin(angle_rad * 0.5f);
    return {std::cos(angle_rad * 0.5f), u.x * s, u.y * s, u.z * s};
  }

  /// Builds an attitude from ZYX Euler angles in radians.
  static Quatf from_euler(float roll, float pitch, float yaw) {
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f);
    const float sy = std::sin(yaw * 0.5f);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
  }

  /// Hamilton product: (a * b) applies b first, then a.
  constexpr Quatf operator*(const Quatf& q) const {
    return {w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x, w * q.z + x * q.y - y * q.x + z * q.w};
  }

  /// The inverse rotation (for a unit quaternion).
  constexpr Quatf conjugate() const { return {w, -x, -y, -z}; }

  constexpr Vector3f imag() const { return {x, y, z}; }

  float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

  Quatf normalized() const {
    const float n = norm();
    return {w / n, x / n, y / n, z / n};
  }

  /// q and -q are the same rotation; the canonical one has w >= 0, which
  /// makes an attitude error take the short way round.
  constexpr Quatf canonical() const { return w < 0.f ? Quatf{-w, -x, -y, -z} : *this; }

  /// Rotates v by this quaternion: body -> world for a vehicle attitude.
  constexpr Vector3f rotate(const Vector3f& v) const {
    const Quatf p{0.f, v.x, v.y, v.z};
    return ((*this) * p * conjugate()).imag();
  }

  /// Returns (roll, pitch, yaw) in radians.
  Vector3f to_euler() const {
    const float roll = std::atan2(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y));
    const float pitch = std::asin(std::clamp(2.f * (w * y - z * x), -1.f, 1.f));
    const float yaw = std::atan2(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z));
    return {roll, pitch, yaw};
  }

  /// Advances the attitude by body angular rate `omega` (rad/s) for dt seconds.
  /// Right-multiplying applies the rotation in the body frame.
  Quatf integrate(const Vector3f& omega, float dt) const {
    const float angle = omega.norm() * dt;
    if (angle < 1e-9f) {
      return *this;
    }
    return ((*this) * from_axis_angle(omega, angle)).normalized();
  }
};

}  // namespace px4p::math
