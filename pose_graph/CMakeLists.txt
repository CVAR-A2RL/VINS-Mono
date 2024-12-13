cmake_minimum_required(VERSION 3.5)
project(pose_graph)

# Configuración de compilación
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(camera_model REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)

# Incluir directorios
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Archivos fuente
add_executable(pose_graph
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
)

# Enlace de bibliotecas
target_link_libraries(pose_graph
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
)

ament_target_dependencies(pose_graph
    rclcpp
    std_msgs
    nav_msgs
    cv_bridge
    OpenCV
    Ceres
    Eigen3
    visualization_msgs
    camera_model
)

# Instalación del ejecutable
install(TARGETS
    pose_graph
    DESTINATION lib/${PROJECT_NAME}
)

# Crear el paquete
ament_package()

