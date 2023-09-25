cmake_minimum_required(VERSION 3.5)
project(orbslam)

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# find dependencies
find_package(ament_cmake REQUIRED)
# .............................................................
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(tf2_geometry_msgs )

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

include_directories(
  ${ORB_SLAM3_ROOT_DIR}/include/
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${ORB_SLAM3_INCLUDE_DIR}
  # ${PCL_INCLUDE_DIRS}
  ${Sophus_DIR}
  ${EIGEN3_INCLUDE_DIR}
  # ${Pangolin_INCLUDE_DIRS}
  )

add_executable(orbslam 
src/main.cpp 
src/slam.cpp
)

ament_target_dependencies(orbslam 
rclcpp sensor_msgs interface cv_bridge ORB_SLAM3 Pangolin geometry_msgs pcl_conversions rcutils message_filters
)

# ament_target_dependencies(
# slam 
# rclcpp sensor_msgs interface cv_bridge ORB_SLAM3 Pangolin geometry_msgs 
# )
# target_link_libraries(slam ${PCL_LIBRARIES})

# target_link_libraries(slam
#   ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
# )

install(TARGETS
orbslam
DESTINATION
lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
