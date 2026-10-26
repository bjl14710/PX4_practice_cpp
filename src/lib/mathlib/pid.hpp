#pragma once

// PID controller for one axis (compare PX4's src/lib/rate_control).

#include <algorithm>

namespace px4p::math {

struct PidGains {
  float kp{0.f};
  float ki{0.f};
  float kd{0.f};
  float integrator_limit{0.f};  ///< |integral term| is clamped to this
  float output_limit{1.f};      ///< |output| is clamped to this
};

class Pid {
 public:
  explicit Pid(const PidGains& gains) : gains_(gains) {}

  /// One control step.
  ///
  /// The D term acts on the measurement, not the error: when the setpoint
  /// jumps, the error jumps too, and differentiating it would kick the
  /// output ("derivative kick"). The measurement is continuous, so it doesn't.
  ///
  /// @param integrate pass false to freeze the integrator (e.g. on the ground)
  float update(float setpoint, float measurement, float dt, bool integrate = true) {
    const float error = setpoint - measurement;

    float derivative = 0.f;
    if (has_previous_ && dt > 0.f) {
      derivative = -(measurement - previous_measurement_) / dt;
    }
    previous_measurement_ = measurement;
    has_previous_ = true;

    if (integrate) {
      integral_ += gains_.ki * error * dt;
      integral_ = std::clamp(integral_, -gains_.integrator_limit, gains_.integrator_limit);
    }

    const float output = gains_.kp * error + integral_ + gains_.kd * derivative;
    return std::clamp(output, -gains_.output_limit, gains_.output_limit);
  }

  void reset() {
    integral_ = 0.f;
    has_previous_ = false;
  }

  float integral() const { return integral_; }
  const PidGains& gains() const { return gains_; }

 private:
  PidGains gains_;
  float integral_{0.f};
  float previous_measurement_{0.f};
  bool has_previous_{false};
};

}  // namespace px4p::math
