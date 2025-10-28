cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch urdf worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
