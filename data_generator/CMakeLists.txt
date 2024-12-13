cmake_minimum_required(VERSION 3.5)
project(data_generator)

# Enable C++11 (or later)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)

# Declare the executable and its sources
add_executable(data_generator
    src/data_generator_node.cpp
    src/data_generator.cpp
)

# Link dependencies
target_link_libraries(data_generator
    ${rclcpp_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

ament_target_dependencies(data_generator
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    cv_bridge
    image_transport
    OpenCV
    visualization_msgs
)

# Specify include directories
target_include_directories(data_generator
    PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Install targets
install(TARGETS
  data_generator
  DESTINATION lib/${PROJECT_NAME}
)

# Create the package
ament_package()

