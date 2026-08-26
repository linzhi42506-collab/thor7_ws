cmake_minimum_required(VERSION 3.22)
project(thor7v3_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

execute_process(
  COMMAND
    ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/sync_thor7_safe_pose.py
    --check
  RESULT_VARIABLE THOR7_SAFE_POSE_CHECK_RESULT
  OUTPUT_VARIABLE THOR7_SAFE_POSE_CHECK_OUTPUT
  ERROR_VARIABLE THOR7_SAFE_POSE_CHECK_ERROR
)

if(NOT THOR7_SAFE_POSE_CHECK_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Thor7 safe-pose synchronization check failed.\n"
    "${THOR7_SAFE_POSE_CHECK_OUTPUT}"
    "${THOR7_SAFE_POSE_CHECK_ERROR}\n"
    "Run:\n"
    "  /usr/bin/python3 "
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/"
    "sync_thor7_safe_pose.py --write")
endif()

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/sync_thor7_safe_pose.py
    scripts/thor7_preflight_gate
  DESTINATION lib/${PROJECT_NAME}
)
