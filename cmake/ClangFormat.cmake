function(ntp_add_clang_format target)
  if(NOT ARGN)
    message(FATAL_ERROR "ntp_add_clang_format: no files provided")
  endif()
  find_program(CLANG_FORMAT NAMES clang-format)
  if(CLANG_FORMAT)
    add_custom_target(${target}_clang_format ALL
      COMMAND ${CLANG_FORMAT} -i -style=file ${ARGN}
      COMMENT "Running clang-format on ${target} sources")
    add_dependencies(${target} ${target}_clang_format)
  else()
    message(STATUS "clang-format not found; skipping source formatting for ${target}")
  endif()
endfunction()

