#pragma once

#include <array>
#include <cstdint>

namespace px4p::msg {

/// Per-motor commands (output of the control allocator).
struct ActuatorMotors {
  static constexpr int kNumMotors = 4;

  uint64_t timestamp{0};                     ///< [us]
  std::array<float, kNumMotors> control{};   ///< normalized [0, 1], PX4 motor order
};

}  // namespace px4p::msg
