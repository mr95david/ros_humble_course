cmake_minimum_required(VERSION 3.8)
project(robot_control_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Paquetes requeridos para ejecutar cada uno de los nodos construidoss
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Instalacion de requeridos de python
ament_python_install_package(${PROJECT_NAME})

# Creamos la funcion que instala un nuevo paquete
# Esta funcion permite agregar el paquete (config) recientemente creado, en el paquete share de ros2
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Instalacion de paquetes anexados
# Esta es la manera correcta de instalar las librerias creadas en python
install(
  PROGRAMS
  ${PROJECT_NAME}/simple_controller_speed.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
