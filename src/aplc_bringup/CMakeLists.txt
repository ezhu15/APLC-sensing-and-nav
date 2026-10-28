cmake_minimum_required(VERSION 3.8)
project(aplc_bringup)
find_package(ament_cmake REQUIRED)
# maps/ is a working folder for RTAB-Map databases in the source tree; it is not installed.
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
ament_package()
