cmake_minimum_required(VERSION 3.20)

project(ntpserver VERSION 0.1.0 LANGUAGES CXX)

option(NTP_SERVER_BUILD_SHARED "Build ntpserver as shared library" OFF)
option(NTP_SERVER_BUILD_EXAMPLE "Build example server program" OFF)
option(NTP_SERVER_BUILD_TESTS "Build unit tests with GoogleTest" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB NTPSERVER_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ntpserver/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ntpserver/**/*.hpp"
)

set(NTPSERVER_SOURCES
    src/ntp_server.cc
    src/qpc_clock.cc
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

    set(CLANG_FORMAT_FILES ${NTPSERVER_SOURCES_ABS} ${NTPSERVER_HEADERS})
    if(NTP_SERVER_BUILD_EXAMPLE)
        list(APPEND CLANG_FORMAT_FILES "${CMAKE_CURRENT_SOURCE_DIR}/example/example_main.cc")
    endif()
    if(NTP_SERVER_BUILD_TESTS)
        file(GLOB TEST_FORMAT_FILES CONFIGURE_DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cc"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.hpp")
        list(APPEND CLANG_FORMAT_FILES ${TEST_FORMAT_FILES})
    endif()

    add_custom_target(ntpserver_clang_format ALL
        COMMAND ${CLANG_FORMAT} -i -style=file ${CLANG_FORMAT_FILES}
        COMMENT "Running clang-format on ntpserver sources")

    add_dependencies(ntpserver ntpserver_clang_format)
    if(TARGET ntpserver_example)
        add_dependencies(ntpserver_example ntpserver_clang_format)
    endif()
else()
    message(STATUS "clang-format not found; skipping source formatting")
endif()

# ----------------------------------------------------------------------------
# Tests (GoogleTest via FetchContent)
# ----------------------------------------------------------------------------
if(NTP_SERVER_BUILD_TESTS)
    include(FetchContent)
    # Clone googletest from GitHub
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    # For MSVC multi-config generators, prevent install rules
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(ntpserver_tests
        tests/ntp_server_test.cc
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
find_program(CPPLINT NAMES cpplint)
if(CPPLINT)
    set(LINT_FILES ${NTPSERVER_SOURCES} ${NTPSERVER_HEADERS})
    if(NTP_SERVER_BUILD_EXAMPLE)
        list(APPEND LINT_FILES example/example_main.cc)
    endif()
    if(NTP_SERVER_BUILD_TESTS)
        list(APPEND LINT_FILES tests/ntp_server_test.cc)
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
