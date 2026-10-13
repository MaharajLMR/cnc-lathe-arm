cmake_minimum_required(VERSION 3.14)
project(lathe_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
