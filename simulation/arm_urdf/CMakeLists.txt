cmake_minimum_required(VERSION 3.8)
project(arm_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS scripts/validate_delivery.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
