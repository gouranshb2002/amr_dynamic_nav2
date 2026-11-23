cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch
  DESTINATION share/amr_description
)

ament_package()
