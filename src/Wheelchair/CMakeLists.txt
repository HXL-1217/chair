cmake_minimum_required(VERSION 3.10)
project(dsw_chair)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# option(USE_ROS2 "Build with ROS2 odometry publisher" OFF)

# Subdirectories
add_subdirectory(lib/SCServo)
add_subdirectory(lib/MotorDevice)
add_subdirectory(lib/UdpComm)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/SCServo
)

# Common libs
set(COMMON_LIBS MotorDevice UdpComm SCServo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dsw_chair
mainROS2.cpp 
)

add_executable(voice_profile_serial_node voice_profile_serial_node.cpp)

target_link_libraries(dsw_chair
    ${COMMON_LIBS}
    rclcpp::rclcpp
    tf2_ros::tf2_ros
)
# target_include_directories(DSWChairROS2 PRIVATE
#     ${nav_msgs_INCLUDE_DIRS}
#     ${geometry_msgs_INCLUDE_DIRS}
# )
ament_target_dependencies(voice_profile_serial_node
  rclcpp
  std_msgs
)

ament_target_dependencies(dsw_chair 
    rclcpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    std_msgs
)

install(TARGETS dsw_chair
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  voice_profile_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    rs485_joystick_node.py
    # cmd_vel_publisher_node.py
    # uHandle.py
    DESTINATION lib/${PROJECT_NAME}
)

# Export/ament package so colcon installs package resources to install/share/<pkg>
ament_package()