cmake_minimum_required(VERSION 3.5)
project(model3)

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
find_package(interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)

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

include_directories(${EIGEN3_INCLUDE_DIR} ${Sophus_DIR} ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
# include_directories(${EIGEN3_INCLUDE_DIR} ${Sophus_DIR} ${PCL_INCLUDE_DIRS})

add_executable(model3 src/main.cpp)

ament_target_dependencies(model3 rclcpp interface cv_bridge pcl_conversions octomap_msgs octomap sensor_msgs) 

# target_link_libraries(model ${OpenCV_LIBS} ${EIGEN_LIBS} ${PCL_LIBRARIES} )
target_link_libraries(model3 ${OpenCV_LIBS} ${EIGEN_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

install(TARGETS
  model3
  DESTINATION
  lib/${PROJECT_NAME}
)

ament_package()
