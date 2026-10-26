#pragma once

// High-resolution timer, named after PX4's drivers/drv_hrt.h.
//
// In PX4 SITL "lockstep" mode the simulator owns time: the flight stack only
// sees time advance when the simulator says so. We do the same thing here,
// which makes every run deterministic and lets tests fly 10 s in milliseconds.

#include <cstdint>

namespace px4p {

/// Absolute time in microseconds since boot (PX4 uses the same type name).
using hrt_abstime = uint64_t;

/// Current lockstep time in microseconds.
hrt_abstime hrt_absolute_time();

namespace hrt {

/// Sets the lockstep clock. Only the simulation loop (FlightStack) calls this.
void set_time(hrt_abstime now_us);

}  // namespace hrt

/// Converts a microsecond interval to seconds as a float.
constexpr float us_to_s(hrt_abstime us) { return static_cast<float>(us) * 1e-6f; }

}  // namespace px4p
