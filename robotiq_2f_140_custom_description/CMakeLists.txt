cmake_minimum_required(VERSION 3.8)
project(robotiq_2f_140_custom_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

