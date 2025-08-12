# need to double check what ubuntu 24 comes with
cmake_minimum_required(VERSION 3.26)
project(swiftnav_ros2_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(tf2 REQUIRED)


# Generated messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Baseline.msg"
  DEPENDENCIES std_msgs
)

# Executable
add_executable(sbp-to-ros
  src/sbp-to-ros.cpp
  src/data_sources/sbp_file_datasource.cpp
  src/data_sources/sbp_serial_datasource.cpp
  src/data_sources/sbp_tcp_datasource.cpp
  src/data_sources/sbp_data_sources.cpp
  src/data_sources/serial.cpp
  src/data_sources/tcp.cpp
  src/utils/utils.cpp
  src/utils/config.cpp
  src/logging/ros_logger.cpp
  src/logging/sbp_to_ros2_logger.cpp
  src/logging/sbp_file_logger.cpp
  src/publishers/navsatfix_publisher.cpp
  src/publishers/twistwithcovariancestamped_publisher.cpp
  src/publishers/timereference_publisher.cpp
  src/publishers/gpsfix_publisher.cpp
  src/publishers/baseline_publisher.cpp
  src/publishers/imu_publisher.cpp
  src/publishers/publisher_factory.cpp
)

# This might have adverse effects, but it is needed for the sbp library
link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")

# Get C++ typesupport target for this package (Jazzy-friendly)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(sbp-to-ros
  sbp
  serialport
  "${cpp_typesupport_target}"
)

target_include_directories(sbp-to-ros PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(sbp-to-ros PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(sbp-to-ros
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  gps_msgs
  tf2
)

# Make generated interfaces available to downstream packages
ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  sbp-to-ros
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
