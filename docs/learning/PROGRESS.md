# Learning Progress

Shared by `/teach` and `/drill-me`. Updated at the end of every session.

## Learner profile
- Goal: work on flight software (satellites, drones) in C++; this repo is the
  PX4-style practice ground.
- Background: has written flight code in C (`flight_and_simulation_basic_satellite`).
  C++-specific idioms and PX4 architecture are the focus.

## Suggested learning path
Ordered so each lesson builds on the previous one, all grounded in this repo:

1. uORB publish/subscribe: `src/uorb/` (templates, singletons, generation counters)
2. Work queues and lockstep time: `src/platform/` (virtual dispatch, scheduling)
3. Frames and quaternions: `src/lib/mathlib/quaternion.hpp` (NED/FRD, body → world)
4. The cascaded control loop: `mc_att_control` → `mc_rate_control` (P vs PID, rate limits)
5. Control allocation and saturation: `control_allocator` (mixer, priorities)
6. Complementary filter: `attitude_estimator` (why accelerometers lie on multirotors)
7. Commander state machine and preflight checks
8. RAII, `= delete`, `final`, `constexpr`, `std::string_view` as used across the repo

## Sessions
_None yet._

## Solid
_Nothing confirmed yet (a concept is "solid" after it holds up in 2+ drills)._

## Needs review
_None yet._

## Next up
Lesson 1: uORB publish/subscribe.
