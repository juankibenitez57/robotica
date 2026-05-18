cmake_minimum_required(VERSION 3.8)
project(tb3_description)

find_package(ament_cmake REQUIRED)

# Instala los ficheros URDF/Xacro para que sean accesibles en runtime
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
