cmake_minimum_required(VERSION 3.5)
project(feature_tracker)

# Establecer el estándar de C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(camera_model REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)


message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")


# Declarar el ejecutable y sus fuentes
add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
)

# Incluir directorios de cabeceras
target_include_directories(feature_tracker
    PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Enlazar bibliotecas necesarias
target_link_libraries(feature_tracker
    ${rclcpp_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)

ament_target_dependencies(feature_tracker
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
    OpenCV
    Eigen3
    message_filters
    camera_model
)

# Instalar los ejecutables
install(TARGETS
  feature_tracker
  DESTINATION lib/${PROJECT_NAME}
)

# Crear el paquete
ament_package()

