cmake_minimum_required(VERSION 3.8)
project(follow_carrot)

# Encuentra dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Crear ejecutable
add_executable(follow_carrot_node src/follow_carrot.cpp)

# Includes
include_directories(
include
)

# Incluir directorios
target_include_directories(follow_carrot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Enlazar librerías
ament_target_dependencies(follow_carrot_node
  rclcpp
  geometry_msgs
)

# Instalar
install(TARGETS follow_carrot_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()