cmake_minimum_required(VERSION 3.8)
project(aplc_bringup)
find_package(ament_cmake REQUIRED)
# maps/ is a working folder for RTAB-Map databases in the source tree; it is not installed.
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/map_alignment.py DESTINATION lib/${PROJECT_NAME} RENAME map_alignment)
install(PROGRAMS scripts/localization_monitor.py DESTINATION lib/${PROJECT_NAME} RENAME localization_monitor)
ament_package()
