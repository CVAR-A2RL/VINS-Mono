cmake_minimum_required(VERSION 3.5)
project(ar_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(camera_model REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ar_demo_node src/ar_demo_node.cpp)

target_include_directories(ar_demo_node
  PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ar_demo_node
  ${rclcpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(ar_demo_node
  rclcpp
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  OpenCV
  message_filters
  camera_model
  Boost
  visualization_msgs
  nav_msgs
  sensor_msgs
)

install(TARGETS
  ar_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
