cmake_minimum_required(VERSION 3.8)
project(robot_description_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Se debe agregar la instalacion de los directorios creados para la descripcion del urdf
# En la seccion de DIRECTORY: se agregan las carpetas creadas
# En la seccion de DESTINATION: Se agrega la ruta final donde se almacenara la carpeta compartida durante la ejecucion de ros2
install(
  DIRECTORY meshes urdf models launch rviz # Donde se almacenaran los directorios creados en el presente paquete.
  DESTINATION share/${PROJECT_NAME}
)

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
