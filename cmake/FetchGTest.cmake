include(FetchContent)

set(NTP_GTEST_ARCHIVE "" CACHE FILEPATH
    "Use a pre-downloaded googletest archive (.zip/.tar.gz) instead of fetching")

function(ntp_fetch_gtest)
  set(options)
  set(oneValueArgs TAG)
  set(multiValueArgs)
  cmake_parse_arguments(NTPG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT NTPG_TAG)
    set(NTPG_TAG v1.17.0)
  endif()

  if(NTP_GTEST_ARCHIVE)
    FetchContent_Declare(
      googletest
      URL ${NTP_GTEST_ARCHIVE}
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
  else()
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG ${NTPG_TAG}
      GIT_SHALLOW TRUE
    )
  endif()

  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endfunction()
