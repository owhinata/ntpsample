cmake_minimum_required(VERSION 3.20)

project(ntpserver VERSION 0.1.0 LANGUAGES CXX)

option(NTP_SERVER_BUILD_SHARED "Build ntpserver as shared library" OFF)
option(NTP_SERVER_BUILD_EXAMPLE "Build example server program" ON)
option(NTP_SERVER_BUILD_TESTS "Build unit tests with GoogleTest" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB NTPSERVER_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ntpserver/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/internal/*.hpp"
)

# Platform detection and platform-specific sources
if(WIN32)
    set(PLATFORM_DIR "win32")
    set(PLATFORM_LIBS ws2_32)
    set(PLATFORM_TIME_SOURCES src/qpc_clock.cc)
elseif(UNIX AND NOT APPLE)
    set(PLATFORM_DIR "posix")
    set(PLATFORM_LIBS "")  # POSIX uses standard libc
    set(PLATFORM_TIME_SOURCES src/platform/posix/monotonic_clock.cc)
elseif(APPLE)
    set(PLATFORM_DIR "posix")
    set(PLATFORM_LIBS "")
    set(PLATFORM_TIME_SOURCES src/platform/posix/monotonic_clock.cc)
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

set(NTPSERVER_SOURCES
    src/ntp_server.cc
    src/ntp_server_options.cc
    src/ntp_types.cc
    src/time_spec.cc
    src/platform/${PLATFORM_DIR}/socket_${PLATFORM_DIR}.cc
    src/platform/${PLATFORM_DIR}/default_time_source.cc
    ${PLATFORM_TIME_SOURCES}
)

if(NTP_SERVER_BUILD_SHARED)
    add_library(ntpserver SHARED ${NTPSERVER_SOURCES} ${NTPSERVER_HEADERS})
    target_compile_definitions(ntpserver PRIVATE NTP_SERVER_BUILDING_DLL PUBLIC NTP_SERVER_SHARED)
else()
    add_library(ntpserver STATIC ${NTPSERVER_SOURCES} ${NTPSERVER_HEADERS})
endif()

target_include_directories(ntpserver
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Compiler-specific flags
if(MSVC)
    target_compile_options(ntpserver PRIVATE /W4 /permissive-)
    target_compile_definitions(ntpserver PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
else()
    # GCC/Clang flags
    target_compile_options(ntpserver PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Platform-specific libraries
if(PLATFORM_LIBS)
    target_link_libraries(ntpserver PRIVATE ${PLATFORM_LIBS})
endif()

# Optional example (off by default)
if(NTP_SERVER_BUILD_EXAMPLE)
    add_executable(ntpserver_example example/example_main.cc)
    target_link_libraries(ntpserver_example PRIVATE ntpserver)
endif()

# ----------------------------------------------------------------------------
# clang-format (auto-run before build if available)
# ----------------------------------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangFormat.cmake)
set(_fmt_files)
foreach(f IN LISTS NTPSERVER_SOURCES NTPSERVER_HEADERS)
  if(NOT IS_ABSOLUTE "${f}")
    list(APPEND _fmt_files "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
  else()
    list(APPEND _fmt_files "${f}")
  endif()
endforeach()
if(NTP_SERVER_BUILD_EXAMPLE)
  list(APPEND _fmt_files "${CMAKE_CURRENT_SOURCE_DIR}/example/example_main.cc")
endif()
if(NTP_SERVER_BUILD_TESTS)
  file(GLOB _test_fmt_files CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.hpp")
  list(APPEND _fmt_files ${_test_fmt_files})
endif()
ntp_add_clang_format(ntpserver ${_fmt_files})

# ----------------------------------------------------------------------------
# Tests (GoogleTest via FetchContent)
# ----------------------------------------------------------------------------
if(NTP_SERVER_BUILD_TESTS)
    include(${CMAKE_CURRENT_LIST_DIR}/../cmake/FetchGTest.cmake)
    ntp_fetch_gtest(TAG v1.14.0)

    enable_testing()
    add_executable(ntpserver_tests
        tests/ntp_server_test.cc
        tests/ntp_extension_test.cc
        tests/time_spec_test.cc
    )
    target_link_libraries(ntpserver_tests PRIVATE ntpserver gtest_main)
    target_include_directories(ntpserver_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if(MSVC)
        target_compile_definitions(ntpserver_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
        target_compile_options(ntpserver_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(ntpserver_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME ntpserver_tests COMMAND ntpserver_tests)
endif()

# ----------------------------------------------------------------------------
# cpplint (run at build if available)
# ----------------------------------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/Cpplint.cmake)
set(_lint_files)
foreach(f IN LISTS NTPSERVER_SOURCES NTPSERVER_HEADERS)
  if(NOT IS_ABSOLUTE "${f}")
    list(APPEND _lint_files "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
  else()
    list(APPEND _lint_files "${f}")
  endif()
endforeach()
if(NTP_SERVER_BUILD_EXAMPLE)
  list(APPEND _lint_files "${CMAKE_CURRENT_SOURCE_DIR}/example/example_main.cc")
endif()
if(NTP_SERVER_BUILD_TESTS)
  file(GLOB _lint_test_files CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.hpp")
  list(APPEND _lint_files ${_lint_test_files})
endif()
ntp_add_cpplint(ntpserver ${_lint_files})
