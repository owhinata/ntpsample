cmake_minimum_required(VERSION 3.20)

project(ntpclock LANGUAGES CXX)

option(NTP_CLOCK_BUILD_SHARED "Build ntpclock as shared library" OFF)
option(NTP_CLOCK_BUILD_EXAMPLE "Build ntpclock example program" ON)
option(NTP_CLOCK_BUILD_TESTS "Build ntpclock unit tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB NTPCLK_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/ntpclock/*.hpp"
)
set(NTPCLK_SOURCES
  src/clock_service.cc
)

if(NTP_CLOCK_BUILD_SHARED)
  add_library(ntpclock SHARED ${NTPCLK_HEADERS} ${NTPCLK_SOURCES})
else()
  add_library(ntpclock STATIC ${NTPCLK_HEADERS} ${NTPCLK_SOURCES})
endif()

target_include_directories(ntpclock PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(ntpclock PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../ntpserver/include)
target_include_directories(ntpclock PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(MSVC)
  target_compile_options(ntpclock PRIVATE /W4 /permissive-)
  target_compile_definitions(ntpclock PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

target_link_libraries(ntpclock PRIVATE ws2_32 ntpserver)

# Formatting / Lint
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangFormat.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/Cpplint.cmake)
set(_fmt)
foreach(f IN LISTS NTPCLK_SOURCES NTPCLK_HEADERS)
  if(NOT IS_ABSOLUTE "${f}")
    list(APPEND _fmt "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
  else()
    list(APPEND _fmt "${f}")
  endif()
endforeach()
ntp_add_clang_format(ntpclock ${_fmt})
ntp_add_cpplint(ntpclock ${_fmt})

# Example program (optional)
if(NTP_CLOCK_BUILD_EXAMPLE)
  add_executable(ntpclock_example example/example_main.cc)
  target_link_libraries(ntpclock_example PRIVATE ntpclock)
endif()

# Tests
if(NTP_CLOCK_BUILD_TESTS)
  include(${CMAKE_CURRENT_LIST_DIR}/../cmake/FetchGTest.cmake)
  ntp_fetch_gtest(TAG v1.14.0)
  enable_testing()
  add_executable(ntpclock_tests tests/clock_service_test.cc)
  target_link_libraries(ntpclock_tests PRIVATE ntpclock gtest gtest_main)
  if(MSVC)
    target_compile_options(ntpclock_tests PRIVATE /W4 /permissive-)
    target_compile_definitions(ntpclock_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
  endif()
  add_test(NAME ntpclock_tests COMMAND ntpclock_tests)
endif()
