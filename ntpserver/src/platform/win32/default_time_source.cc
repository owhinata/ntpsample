// Copyright (c) 2025
/**
 * @file default_time_source.cc (Windows)
 * @brief Windows implementation - returns QpcClock singleton.
 */
#include "ntpserver/platform/default_time_source.hpp"

#include "ntpserver/qpc_clock.hpp"

namespace ntpserver {
namespace platform {

TimeSource& GetDefaultTimeSource() { return QpcClock::Instance(); }

}  // namespace platform
}  // namespace ntpserver
