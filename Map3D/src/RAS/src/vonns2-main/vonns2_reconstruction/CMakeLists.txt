cmake_minimum_required(VERSION 3.8)
project(vonns2_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Defining used ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)

# sudo apt install pcl* pcl-tools libpcl-dev libpcap-dev
# Installing Point Cloud Library
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Creating C++ executables and add them their implemented ROS2 packages

add_executable(point_cloud2_assembler src/point_cloud2_assembler.cpp)
target_link_libraries(point_cloud2_assembler ${PCL_LIBRARIES})
ament_target_dependencies(point_cloud2_assembler rclcpp sensor_msgs PCL pcl_conversions)

add_executable(point_cloud2_converter src/point_cloud2_converter.cpp)
ament_target_dependencies(point_cloud2_converter rclcpp sensor_msgs tf2_ros laser_geometry)

# Installing custom C++ executables in ROS2 ecosystem
install(TARGETS
  point_cloud2_assembler
  point_cloud2_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Installing custom Python codes in ROS2 ecosystem
ament_python_install_package(scripts)
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

# Installing additional directories for the ROS2 package
install(
  DIRECTORY config launch pcd
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
