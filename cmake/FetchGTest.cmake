include(FetchContent)

function(ntp_fetch_gtest)
  set(options)
  set(oneValueArgs TAG)
  set(multiValueArgs)
  cmake_parse_arguments(NTPG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT NTPG_TAG)
    set(NTPG_TAG v1.14.0)
  endif()

  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG ${NTPG_TAG}
    GIT_SHALLOW TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endfunction()
