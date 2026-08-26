cmake_minimum_required(VERSION 3.8)
project(thor7v3_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES urdf.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
