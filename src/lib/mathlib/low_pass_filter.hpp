#pragma once

// First-order low-pass filter (compare PX4's AlphaFilter).
// A template, so the same code filters a float or a whole Vector3f.

#include "lib/mathlib/vector3.hpp"

namespace px4p::math {

template <typename T>
class LowPassFilter {
 public:
  /// @param sample_hz how often apply() is called
  /// @param cutoff_hz frequency where the filter starts attenuating
  LowPassFilter(float sample_hz, float cutoff_hz) {
    constexpr float kTwoPi = 6.28318531f;
    const float dt = 1.f / sample_hz;
    const float rc = 1.f / (kTwoPi * cutoff_hz);
    alpha_ = dt / (rc + dt);
  }

  /// Feeds one sample and returns the filtered value.
  /// The first sample initializes the state so the output doesn't ramp from zero.
  T apply(const T& sample) {
    if (!initialized_) {
      state_ = sample;
      initialized_ = true;
    } else {
      state_ = state_ + (sample - state_) * alpha_;
    }
    return state_;
  }

  const T& state() const { return state_; }
  float alpha() const { return alpha_; }

 private:
  float alpha_{1.f};
  T state_{};
  bool initialized_{false};
};

}  // namespace px4p::math
