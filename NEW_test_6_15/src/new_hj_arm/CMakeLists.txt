cmake_minimum_required(VERSION 3.5)
project(new_hj_arm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
    meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY
    urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
