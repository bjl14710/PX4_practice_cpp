// px4p_sitl: flies the demo script in software-in-the-loop and logs to CSV.
//
// Usage: px4p_sitl [log.csv]

#include <cstdio>
#include <fstream>
#include <iostream>

#include "flight_stack.hpp"
#include "lib/mathlib/constants.hpp"
#include "modules/flight_script/flight_script.hpp"
#include "modules/logger/logger.hpp"

int main(int argc, char** argv) {
  using namespace px4p;

  const char* log_path = argc > 1 ? argv[1] : "px4p_log.csv";
  std::ofstream log_file(log_path);
  if (!log_file) {
    std::cerr << "cannot open " << log_path << " for writing\n";
    return 1;
  }

  FlightStack stack;
  FlightScript pilot(demo_flight(stack.sim().hover_command()));
  Logger logger(log_file, stack.sim());
  stack.add(pilot);
  stack.add(logger);

  std::printf("%6s %6s %8s %8s %8s %8s\n", "t[s]", "armed", "roll", "pitch", "yaw", "alt[m]");
  constexpr hrt_abstime kEndUs = 18'000'000;
  constexpr hrt_abstime kPrintEveryUs = 1'000'000;
  while (stack.now() < kEndUs) {
    stack.run_until(stack.now() + kPrintEveryUs);
    const math::Vector3f e = stack.sim().state().attitude.to_euler();
    const bool armed = stack.commander().arming_state() == msg::VehicleStatus::ArmingState::kArmed;
    std::printf("%6.1f %6s %8.2f %8.2f %8.2f %8.2f\n", static_cast<double>(us_to_s(stack.now())),
                armed ? "yes" : "no", static_cast<double>(math::rad_to_deg(e.x)),
                static_cast<double>(math::rad_to_deg(e.y)),
                static_cast<double>(math::rad_to_deg(e.z)),
                static_cast<double>(stack.sim().altitude_m()));
  }

  std::cout << "log written to " << log_path << '\n';
  return 0;
}
