# SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)

#########################################################################
# Run tests under Valgrind

cmake_dependent_option(
  YARP_VALGRIND_TESTS
  "Run YARP tests under Valgrind" OFF
  "YARP_COMPILE_TESTS" OFF
)
mark_as_advanced(YARP_VALGRIND_TESTS)

if(YARP_VALGRIND_TESTS)
  find_program(VALGRIND_EXECUTABLE NAMES valgrind)
  mark_as_advanced(VALGRIND_EXECUTABLE)

  if(VALGRIND_EXECUTABLE)
    set(VALGRIND_OPTIONS "--tool=memcheck --leak-check=full"
      CACHE STRING "Valgrind options (--error-exitcode=1 will be appended)")
    mark_as_advanced(VALGRIND_OPTIONS)
    separate_arguments(VALGRIND_OPTIONS UNIX_COMMAND "${VALGRIND_OPTIONS}")
    set(VALGRIND_COMMAND "${VALGRIND_EXECUTABLE}" ${VALGRIND_OPTIONS} --error-exitcode=1 --fullpath-after=${CMAKE_SOURCE_DIR}/)
  else()
    message(SEND_ERROR "Valgrind executable not found")
  endif()
endif()

unset(YARP_TEST_LAUNCHER)
set(YARP_TEST_TIMEOUT_DEFAULT_VALGRIND 300)
if(DEFINED VALGRIND_COMMAND)
  set(YARP_TEST_LAUNCHER ${VALGRIND_COMMAND})
  # The default timeout is not enough when running under valgrind
  if(YARP_TEST_TIMEOUT EQUAL YARP_TEST_TIMEOUT_DEFAULT)
    set_property(CACHE YARP_TEST_TIMEOUT PROPERTY VALUE ${YARP_TEST_TIMEOUT_DEFAULT_VALGRIND})
  endif()
else()
  if(YARP_TEST_TIMEOUT EQUAL YARP_TEST_TIMEOUT_DEFAULT_VALGRIND)
    set_property(CACHE YARP_TEST_TIMEOUT PROPERTY VALUE ${YARP_TEST_TIMEOUT_DEFAULT})
  endif()
endif()
