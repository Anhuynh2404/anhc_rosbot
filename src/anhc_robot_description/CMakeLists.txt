cmake_minimum_required(VERSION 3.8)
project(anhc_robot_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install all directories
install(DIRECTORY urdf meshes config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
