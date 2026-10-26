#include "flight_stack.hpp"

namespace px4p {

FlightStack::FlightStack(const sim::QuadrotorParams& vehicle) : sim_(vehicle), bridge_(sim_) {
  hrt::set_time(now_us_);

  // Order matters: each module reads what the one before it just published,
  // so data flows through the whole chain within a single tick.
  work_queue_.add(bridge_);
  work_queue_.add(sensors_);
  work_queue_.add(estimator_);
  work_queue_.add(commander_);
  work_queue_.add(att_control_);
  work_queue_.add(rate_control_);
  work_queue_.add(allocator_);
}

void FlightStack::step() {
  work_queue_.run_due(now_us_);
  now_us_ += kTickUs;
  hrt::set_time(now_us_);
}

void FlightStack::run_until(hrt_abstime t_end_us) {
  while (now_us_ < t_end_us) {
    step();
  }
}

}  // namespace px4p
