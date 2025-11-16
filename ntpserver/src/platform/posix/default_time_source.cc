// Copyright (c) 2025
/**
 * @file default_time_source.cc (POSIX)
 * @brief POSIX implementation - returns MonotonicClock singleton.
 */
#include "ntpserver/platform/default_time_source.hpp"

#include "ntpserver/monotonic_clock.hpp"

namespace ntpserver {
namespace platform {

TimeSource& GetDefaultTimeSource() { return MonotonicClock::Instance(); }

}  // namespace platform
}  // namespace ntpserver
