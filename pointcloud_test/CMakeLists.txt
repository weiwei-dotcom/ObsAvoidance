cmake_minimum_required(VERSION 3.5)
project(pointcloud_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(interface REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(octomap)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pointcloud_test src/main.cpp)

ament_target_dependencies(pointcloud_test 
  rclcpp
  sensor_msgs
  octomap_msgs
  interface 
  pcl_conversions
  octomap
)

target_link_libraries(pointcloud_test
  ${PCL_LIBRARIES}
  # ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

install(
  TARGETS
  pointcloud_test
  DESTINATION
  lib/${PROJECT_NAME}
)

ament_package()
