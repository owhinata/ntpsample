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
)

set(NTPSERVER_SOURCES
    src/ntp_server.cc
    src/qpc_clock.cc
    src/ntp_types.cc
    src/time_spec.cc
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

if(MSVC)
    target_compile_options(ntpserver PRIVATE /W4 /permissive-)
    target_compile_definitions(ntpserver PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# Winsock2 on Windows
target_link_libraries(ntpserver PRIVATE ws2_32)

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
    target_link_libraries(ntpserver_tests PRIVATE ntpserver gtest gtest_main ws2_32)
    target_include_directories(ntpserver_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if(MSVC)
        target_compile_definitions(ntpserver_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
        target_compile_options(ntpserver_tests PRIVATE /W4 /permissive-)
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
  list(APPEND _lint_files "${CMAKE_CURRENT_SOURCE_DIR}/tests/ntp_server_test.cc")
endif()
ntp_add_cpplint(ntpserver ${_lint_files})
