// Copyright (c) 2025
/**
 * @file default_time_source.cc (POSIX)
 * @brief POSIX implementation - creates MonotonicClock instance.
 */
#include "ntpserver/platform/default_time_source.hpp"

#include <memory>

#include "ntpserver/monotonic_clock.hpp"

namespace ntpserver {
namespace platform {

std::unique_ptr<TimeSource> CreateDefaultTimeSource() {
  return std::make_unique<MonotonicClock>();
}

}  // namespace platform
}  // namespace ntpserver
