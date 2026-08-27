cmake_minimum_required(VERSION 3.8)
project(arm_uav_joint)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/publish_custom_quad_display.py
    scripts/px4_takeoff.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_arm_uav_joint tests/test_assets.py)
endif()

ament_package()
