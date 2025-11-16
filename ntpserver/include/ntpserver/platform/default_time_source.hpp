// Copyright (c) 2025
/**
 * @file default_time_source.hpp
 * @brief Platform-agnostic access to default TimeSource.
 *
 * Provides a unified way to access the platform's default TimeSource
 * implementation without platform-specific includes in application code.
 */
#pragma once

#include "ntpserver/time_source.hpp"

namespace ntpserver {
namespace platform {

/**
 * @brief Get the default TimeSource for this platform.
 *
 * Returns a reference to the platform-specific default TimeSource:
 * - Windows: QpcClock (QueryPerformanceCounter-based)
 * - POSIX (Linux/macOS): MonotonicClock (clock_gettime-based)
 *
 * The returned instance is a singleton and thread-safe.
 *
 * @return Reference to the default TimeSource instance.
 */
TimeSource& GetDefaultTimeSource();

}  // namespace platform
}  // namespace ntpserver
