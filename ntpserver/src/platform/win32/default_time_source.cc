// Copyright (c) 2025
/**
 * @file default_time_source.cc (Windows)
 * @brief Windows implementation - creates QpcClock instance.
 */
#include "ntpserver/platform/default_time_source.hpp"

#include <memory>

#include "ntpserver/qpc_clock.hpp"

namespace ntpserver {
namespace platform {

std::unique_ptr<TimeSource> CreateDefaultTimeSource() {
  return std::make_unique<QpcClock>();
}

}  // namespace platform
}  // namespace ntpserver
