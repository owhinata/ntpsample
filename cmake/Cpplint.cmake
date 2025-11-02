function(ntp_add_cpplint target)
  if(NOT ARGN)
    message(FATAL_ERROR "ntp_add_cpplint: no files provided")
  endif()
  find_program(CPPLINT NAMES cpplint)
  if(CPPLINT)
    add_custom_target(${target}_cpplint ALL
      COMMAND ${CPPLINT} --filter=-build/c++11 ${ARGN}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Running cpplint on ${target} sources")
    add_dependencies(${target} ${target}_cpplint)
  else()
    message(STATUS "cpplint not found; skipping lint for ${target}")
  endif()
endfunction()

