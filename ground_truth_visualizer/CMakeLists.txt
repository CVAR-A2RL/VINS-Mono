cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME ground_truth_visualizer)
project(${PROJECT_NAME} VERSION 0.1.0)

# Set the C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Add the include directories
include_directories(
  include
  include/${PROJECT_NAME}
)

# Add the source files
set(SOURCE_CPP_FILES
  src/${PROJECT_NAME}.cpp
)

# Create library
# add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})
# ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp ${SOURCE_CPP_FILES})
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
# target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

# Include directories
# target_include_directories(${PROJECT_NAME}_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# Export libraries and targets
# install(
#   TARGETS
#     ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Export directories
# ament_export_include_directories(include)

# Export libraries
# ament_export_libraries(${PROJECT_NAME})

# Install launch and config files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
