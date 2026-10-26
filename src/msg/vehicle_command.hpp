#pragma once

#include <cstdint>

namespace px4p::msg {

/// A command sent to the vehicle (a tiny subset of MAVLink commands).
struct VehicleCommand {
  enum class Command : uint8_t { kNone, kArm, kDisarm };

  uint64_t timestamp{0};          ///< [us]
  Command command{Command::kNone};
};

}  // namespace px4p::msg
