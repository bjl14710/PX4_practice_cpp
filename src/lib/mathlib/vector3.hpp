#pragma once

// Minimal 3-vector, modelled on PX4's matrix::Vector3f (src/lib/matrix).
// Floats, not doubles: flight controllers often only have a single-precision
// FPU, which is why the build uses -Wdouble-promotion.

#include <cmath>

namespace px4p::math {

struct Vector3f {
  float x{0.f};
  float y{0.f};
  float z{0.f};

  constexpr Vector3f() = default;
  constexpr Vector3f(float x_in, float y_in, float z_in) : x(x_in), y(y_in), z(z_in) {}

  constexpr Vector3f operator+(const Vector3f& o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vector3f operator-(const Vector3f& o) const { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vector3f operator-() const { return {-x, -y, -z}; }
  constexpr Vector3f operator*(float s) const { return {x * s, y * s, z * s}; }
  constexpr Vector3f operator/(float s) const { return {x / s, y / s, z / s}; }

  Vector3f& operator+=(const Vector3f& o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }

  Vector3f& operator-=(const Vector3f& o) {
    x -= o.x;
    y -= o.y;
    z -= o.z;
    return *this;
  }

  /// Axis access by index (0 = x, 1 = y, 2 = z), handy for per-axis loops.
  constexpr float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
  float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

  constexpr float dot(const Vector3f& o) const { return x * o.x + y * o.y + z * o.z; }

  constexpr Vector3f cross(const Vector3f& o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }

  /// Element-wise multiply (PX4 calls this emult).
  constexpr Vector3f emult(const Vector3f& o) const { return {x * o.x, y * o.y, z * o.z}; }

  /// Element-wise divide (PX4 calls this edivide).
  constexpr Vector3f edivide(const Vector3f& o) const { return {x / o.x, y / o.y, z / o.z}; }

  float norm() const { return std::sqrt(dot(*this)); }

  /// Unit vector in the same direction, or zero if the vector is ~zero.
  Vector3f normalized() const {
    const float n = norm();
    return n > 1e-6f ? *this / n : Vector3f{};
  }
};

constexpr Vector3f operator*(float s, const Vector3f& v) { return v * s; }

}  // namespace px4p::math
