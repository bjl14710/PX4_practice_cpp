#pragma once

#include "uorb/topic.hpp"

namespace px4p::uorb {

/// Publishes messages of type T (PX4: uORB::Publication<T>).
template <typename T>
class Publication {
 public:
  void publish(const T& msg) { Topic<T>::instance().publish(msg); }
};

}  // namespace px4p::uorb
