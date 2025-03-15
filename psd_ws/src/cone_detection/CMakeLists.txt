cmake_minimum_required(VERSION 3.8)
project(cone_detection)

# Optionally set policies to suppress warnings about _ROOT variables
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Enable OpenMP if found
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# -- STEREOCAMERA DEPTH NODE --
add_executable(cone_detection_stereocam src/cone_detection_stereocam.cpp)
ament_target_dependencies(
  cone_detection_stereocam
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)
target_link_libraries(cone_detection_stereocam
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)
target_compile_definitions(
  cone_detection_stereocam
  PUBLIC ${PCL_DEFINITIONS}
)

# -- LIDAR NODE --
add_executable(cone_detection_lidar src/cone_detection_lidar.cpp)
ament_target_dependencies(
  cone_detection_lidar
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)
target_link_libraries(cone_detection_lidar
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)
target_compile_definitions(
  cone_detection_lidar
  PUBLIC ${PCL_DEFINITIONS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install both executables
install(
  TARGETS
    cone_detection_stereocam
    cone_detection_lidar
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
