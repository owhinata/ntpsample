cmake_minimum_required(VERSION 3.20)

project(ntpserver VERSION 0.1.0 LANGUAGES CXX)

option(NTP_SERVER_BUILD_SHARED "Build ntpserver as shared library" OFF)
option(NTP_SERVER_BUILD_EXAMPLE "Build example server program" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB NTPSERVER_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ntpserver/*.hpp"
)

set(NTPSERVER_SOURCES
    src/ntp_server.cc
    src/user_time.cc
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
find_program(CLANG_FORMAT NAMES clang-format)
if(CLANG_FORMAT)
    # Build absolute file list for formatting
    set(NTPSERVER_SOURCES_ABS)
    foreach(f IN LISTS NTPSERVER_SOURCES)
        if(NOT IS_ABSOLUTE "${f}")
            list(APPEND NTPSERVER_SOURCES_ABS "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
        else()
            list(APPEND NTPSERVER_SOURCES_ABS "${f}")
        endif()
    endforeach()

    add_custom_target(ntpserver_clang_format ALL
        COMMAND ${CLANG_FORMAT} -i -style=file ${NTPSERVER_SOURCES_ABS} ${NTPSERVER_HEADERS}
        COMMENT "Running clang-format on ntpserver sources")

    add_dependencies(ntpserver ntpserver_clang_format)
    if(TARGET ntpserver_example)
        add_dependencies(ntpserver_example ntpserver_clang_format)
    endif()
else()
    message(STATUS "clang-format not found; skipping source formatting")
endif()

# ----------------------------------------------------------------------------
# cpplint (run at build if available)
# ----------------------------------------------------------------------------
find_program(CPPLINT NAMES cpplint)
if(CPPLINT)
    set(LINT_FILES ${NTPSERVER_SOURCES} ${NTPSERVER_HEADERS})
    if(NTP_SERVER_BUILD_EXAMPLE)
        list(APPEND LINT_FILES example/example_main.cc)
    endif()

    # Make file paths absolute for robustness
    set(LINT_FILES_ABS)
    foreach(f IN LISTS LINT_FILES)
        if(NOT IS_ABSOLUTE "${f}")
            list(APPEND LINT_FILES_ABS "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
        else()
            list(APPEND LINT_FILES_ABS "${f}")
        endif()
    endforeach()

    add_custom_target(ntpserver_cpplint ALL
        COMMAND ${CPPLINT} --filter=-build/c++11 ${LINT_FILES_ABS}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Running cpplint on ntpserver sources")

    add_dependencies(ntpserver ntpserver_cpplint)
    if(TARGET ntpserver_example)
        add_dependencies(ntpserver_example ntpserver_cpplint)
    endif()
else()
    message(STATUS "cpplint not found; skipping lint")
endif()
