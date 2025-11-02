cmake_minimum_required(VERSION 3.20)

project(ntpclient LANGUAGES CXX)

option(NTP_CLIENT_BUILD_SHARED "Build ntpclient as shared library" OFF)
option(NTP_CLIENT_BUILD_EXAMPLE "Build ntpclient example program" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB NTPC_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/ntpclient/*.hpp"
)
set(NTPC_SOURCES
  src/ntp_client.cc
)

if(NTP_CLIENT_BUILD_SHARED)
  add_library(ntpclient SHARED ${NTPC_HEADERS} ${NTPC_SOURCES})
else()
  add_library(ntpclient STATIC ${NTPC_HEADERS} ${NTPC_SOURCES})
endif()

target_include_directories(ntpclient PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(ntpclient PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../ntpserver/include)

if(MSVC)
  target_compile_options(ntpclient PRIVATE /W4 /permissive-)
  target_compile_definitions(ntpclient PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

target_link_libraries(ntpclient PRIVATE ws2_32 ntpserver)

if(NTP_CLIENT_BUILD_EXAMPLE)
  add_executable(ntpclient_example example/example_main.cc)
  target_include_directories(ntpclient_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../ntpserver/include)
  target_link_libraries(ntpclient_example PRIVATE ntpclient ntpserver)
endif()

# Formatting and lint
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangFormat.cmake)
set(_fmt)
foreach(f IN LISTS NTPC_SOURCES NTPC_HEADERS)
  if(NOT IS_ABSOLUTE "${f}")
    list(APPEND _fmt "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
  else()
    list(APPEND _fmt "${f}")
  endif()
endforeach()
if(NTP_CLIENT_BUILD_EXAMPLE)
  list(APPEND _fmt "${CMAKE_CURRENT_SOURCE_DIR}/example/example_main.cc")
endif()
ntp_add_clang_format(ntpclient ${_fmt})

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/Cpplint.cmake)
set(_lint ${_fmt})
ntp_add_cpplint(ntpclient ${_lint})
