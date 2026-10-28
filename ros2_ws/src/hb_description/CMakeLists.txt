cmake_minimum_required(VERSION 3.8)
project(hb_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
