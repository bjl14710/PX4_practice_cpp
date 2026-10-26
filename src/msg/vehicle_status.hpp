#pragma once

#include <cstdint>

namespace px4p::msg {

/// Overall vehicle state, owned by the commander.
struct VehicleStatus {
  enum class ArmingState : uint8_t { kDisarmed, kArmed };

  uint64_t timestamp{0};                          ///< [us]
  ArmingState arming_state{ArmingState::kDisarmed};
};

}  // namespace px4p::msg
