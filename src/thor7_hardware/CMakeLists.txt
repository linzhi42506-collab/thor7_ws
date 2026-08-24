cmake_minimum_required(VERSION 3.8)
project(thor7_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------
# Diana SDK 定位：
# 优先使用 CMake 缓存变量 DIANA_API_ROOT，
# 其次读取环境变量 DIANA_API_ROOT
# ------------------------------------------------------------------
if(NOT DEFINED DIANA_API_ROOT)
  if(DEFINED ENV{DIANA_API_ROOT})
    set(DIANA_API_ROOT $ENV{DIANA_API_ROOT})
  endif()
endif()

if(NOT DIANA_API_ROOT)
  message(FATAL_ERROR
    "DIANA_API_ROOT is not set. Set it to the Diana SDK root that contains include/ and lib/, e.g.\n"
    "  export DIANA_API_ROOT=/home/landau/3rdparty/Diana_new12/DianaApi\n"
    "or pass -DDIANA_API_ROOT=... to colcon build.")
endif()

find_path(DIANA_API_INCLUDE_DIR DianaAPI.h
  PATHS
    ${DIANA_API_ROOT}/include
  NO_DEFAULT_PATH
)

find_library(DIANA_API_LIBRARY
  NAMES DianaApi
  PATHS
    ${DIANA_API_ROOT}/lib
    ${DIANA_API_ROOT}/bin
  NO_DEFAULT_PATH
)

if(NOT DIANA_API_INCLUDE_DIR)
  message(FATAL_ERROR
    "DianaAPI.h not found under ${DIANA_API_ROOT}/include. Check DIANA_API_ROOT.")
endif()

if(NOT DIANA_API_LIBRARY)
  message(FATAL_ERROR
    "libDianaApi.so not found under ${DIANA_API_ROOT}/lib or ${DIANA_API_ROOT}/bin. Check DIANA_API_ROOT.")
endif()

message(STATUS "DIANA_API_ROOT        = ${DIANA_API_ROOT}")
message(STATUS "DIANA_API_INCLUDE_DIR = ${DIANA_API_INCLUDE_DIR}")
message(STATUS "DIANA_API_LIBRARY     = ${DIANA_API_LIBRARY}")

# ------------------------------------------------------------------
# ROS 2 dependencies
# ------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# ros2_control hardware plugin
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ------------------------------------------------------------------
# 厂商 serial 库
#
# 大寰官方 DhGripper/third_party/serial，MIT License。
# 只重新编译源码，不使用 vendor 中已有的 .a/.o。
#
# thor7_serial 最终会被链接进共享库 libthor7_system.so，
# 所以必须打开 POSITION_INDEPENDENT_CODE。
# ------------------------------------------------------------------
add_library(thor7_serial STATIC
  vendor/serial/src/serial.cc
  vendor/serial/src/impl/unix.cc
  vendor/serial/src/impl/list_ports/list_ports_linux.cc
)

target_include_directories(thor7_serial
  PUBLIC
    vendor/serial/include
)

target_compile_features(thor7_serial
  PUBLIC
    cxx_std_11
)

target_link_libraries(thor7_serial
  PUBLIC
    pthread
)

set_target_properties(thor7_serial PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# ------------------------------------------------------------------
# 真机只读客户端库
#
# DianaReadonlyClient:
#   Thor7 Diana SDK 只读连接
#
# Ag145ModbusClient:
#   AG145 Modbus/Serial 底层通信
#
# 当前 Thor7 ros2_control 第一阶段仍然严格只读。
# ------------------------------------------------------------------
add_library(thor7_hardware_readonly
  src/diana_readonly_client.cpp
  src/ag145_modbus_client.cpp
)

target_include_directories(thor7_hardware_readonly
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/serial/include
  PRIVATE
    ${DIANA_API_INCLUDE_DIR}
)

target_link_libraries(thor7_hardware_readonly
  PRIVATE
    thor7_serial
    ${DIANA_API_LIBRARY}
)

# libDianaApi.so 依赖的厂商动态库位于 SDK bin/ 目录
target_link_directories(thor7_hardware_readonly
  PRIVATE
    ${DIANA_API_ROOT}/bin
)

# thor7_hardware_readonly 会被链接进共享插件 libthor7_system.so，
# 因此也必须使用 PIC。
set_target_properties(thor7_hardware_readonly PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  BUILD_RPATH "${DIANA_API_ROOT}/lib;${DIANA_API_ROOT}/bin"
  INSTALL_RPATH "${DIANA_API_ROOT}/lib;${DIANA_API_ROOT}/bin"
)

# ------------------------------------------------------------------
# ros2_control Thor7 真机 READ-ONLY SystemInterface
#
# 当前版本：
#   - 只读取 joint1 ~ joint6 position
#   - 不导出 command interface
#   - write() 不调用任何 Diana 运动 API
# ------------------------------------------------------------------
add_library(thor7_system SHARED
  src/thor7_system.cpp
)

ament_target_dependencies(thor7_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_include_directories(thor7_system
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${DIANA_API_INCLUDE_DIR}
)

target_link_libraries(thor7_system
    thor7_hardware_readonly
    ${DIANA_API_LIBRARY}
)

target_link_directories(thor7_system
  PRIVATE
    ${DIANA_API_ROOT}/bin
)

set_target_properties(thor7_system PROPERTIES
  BUILD_RPATH "${DIANA_API_ROOT}/lib;${DIANA_API_ROOT}/bin"
  INSTALL_RPATH "${DIANA_API_ROOT}/lib;${DIANA_API_ROOT}/bin"
)

# 向 pluginlib / ros2_control 注册硬件插件
pluginlib_export_plugin_description_file(
  hardware_interface
  thor7_hardware_plugin.xml
)

# ------------------------------------------------------------------
# Thor7 真机只读 probe
# ------------------------------------------------------------------
add_executable(thor7_readonly_probe
  src/thor7_readonly_probe.cpp
)

ament_target_dependencies(thor7_readonly_probe
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

target_include_directories(thor7_readonly_probe
  PRIVATE
    ${DIANA_API_INCLUDE_DIR}
)

target_link_libraries(thor7_readonly_probe
  thor7_hardware_readonly
)

# ------------------------------------------------------------------
# AG145 真机只读 probe
# ------------------------------------------------------------------
add_executable(ag145_readonly_probe
  src/ag145_readonly_probe.cpp
)

ament_target_dependencies(ag145_readonly_probe
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

target_link_libraries(ag145_readonly_probe
  thor7_hardware_readonly
)

# Thor7 限制参数 probe (新增)
# ------------------------------------------------------------------
add_executable(
  thor7_limits_probe
  src/thor7_limits_probe.cpp
)

target_include_directories(
  thor7_limits_probe
  PRIVATE
    ${DIANA_API_INCLUDE_DIR}
)

target_link_libraries(
  thor7_limits_probe
  thor7_hardware_readonly
  ${DIANA_API_LIBRARY}
)

# ------------------------------------------------------------------
# Thor7 第一次真实单关节运动测试
# 默认 dry-run；只有显式 --execute + 人工输入 MOVE 才调用 moveJToTarget()
# ------------------------------------------------------------------
add_executable(
  thor7_single_joint_test
  src/thor7_single_joint_test.cpp
)

target_include_directories(
  thor7_single_joint_test
  PRIVATE
    ${DIANA_API_INCLUDE_DIR}
)

target_link_libraries(
  thor7_single_joint_test
  ${DIANA_API_LIBRARY}
)

target_link_directories(
  thor7_single_joint_test
  PRIVATE
    ${DIANA_API_ROOT}/bin
)

set_target_properties(
  thor7_single_joint_test
  PROPERTIES
    BUILD_RPATH "${DIANA_API_ROOT}/lib;${DIANA_API_ROOT}/bin"
    INSTALL_RPATH "${DIANA_API_ROOT}/lib;${DIANA_API_ROOT}/bin"
)

# ------------------------------------------------------------------
# 单元测试
#
# 纯函数测试：
# 不连接 Thor7
# 不连接 AG145
# 不依赖 Diana SDK 运行时
# ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_ag145_crc
    test/test_ag145_crc.cpp
  )
  if(TARGET test_ag145_crc)
    target_include_directories(
      test_ag145_crc
      PRIVATE
        include
    )
  endif()

  ament_add_gtest(
    test_ag145_mapping
    test/test_ag145_mapping.cpp
  )
  if(TARGET test_ag145_mapping)
    target_include_directories(
      test_ag145_mapping
      PRIVATE
        include
    )
  endif()

  ament_add_gtest(
    test_ag145_frame
    test/test_ag145_frame.cpp
  )
  if(TARGET test_ag145_frame)
    target_include_directories(
      test_ag145_frame
      PRIVATE
        include
    )
  endif()
endif()

# ------------------------------------------------------------------
# Install
# ------------------------------------------------------------------

# ros2_control hardware plugin
install(
  TARGETS thor7_system
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# readonly probe executables
install(
  TARGETS
    thor7_readonly_probe
    ag145_readonly_probe
    thor7_limits_probe
    thor7_single_joint_test
  DESTINATION lib/${PROJECT_NAME}
)


# public headers
install(
  DIRECTORY include/
  DESTINATION include
)

# launch + config
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# README
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)


# ------------------------------------------------------------------
# Thor7 servoJ 独立验证程序
# ------------------------------------------------------------------
add_executable(thor7_servoj_test
  src/thor7_servoj_test.cpp)

target_include_directories(thor7_servoj_test PRIVATE
  ${DIANA_API_INCLUDE_DIR})

target_link_libraries(thor7_servoj_test
  thor7_hardware_readonly)

install(TARGETS thor7_servoj_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()

