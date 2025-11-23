// Copyright (c) 2025
/**
 * @file default_time_source.hpp
 * @brief Platform-specific default TimeSource factory.
 */
#pragma once

#include <memory>

#include "ntpserver/time_source.hpp"

namespace ntpserver {
namespace platform {

/**
 * @brief Creates a platform-specific default TimeSource.
 * @return Unique pointer to MonotonicClock (POSIX) or QpcClock (Windows).
 */
std::unique_ptr<TimeSource> CreateDefaultTimeSource();

}  // namespace platform
}  // namespace ntpserver
