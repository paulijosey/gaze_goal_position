cmake_minimum_required(VERSION 3.8)
project(gaze_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (ROS)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# find dependencies (not ROS)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio xfeatures2d)
include_directories(${OpenCV_INCLUDE_DIRS})
# make node executable
add_executable(gaze_transform_node src/gaze_transform_node.cpp)
target_include_directories(gaze_transform_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(gaze_viz_node src/gaze_viz_node.cpp)
target_include_directories(gaze_viz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(gaze_transform_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# add dependencies
ament_target_dependencies(gaze_transform_node 
                          rclcpp 
                          std_msgs
                          geometry_msgs
                          sensor_msgs
                          cv_bridge
                          image_transport
                          OpenCV
                          )

ament_target_dependencies(gaze_viz_node 
                          rclcpp 
                          std_msgs
                          geometry_msgs
                          sensor_msgs
                          cv_bridge
                          OpenCV
                          )

install(TARGETS gaze_transform_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS gaze_viz_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
