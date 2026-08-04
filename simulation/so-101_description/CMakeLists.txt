cmake_minimum_required(VERSION 3.8)
project(so-101_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)

# 安装目录
install(
  DIRECTORY
    launch
    rviz
    config
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# 安装 Python 脚本 (executable)
install(PROGRAMS
  scripts/initialize_joint_controller.py
  scripts/plan_execute_smoke_test.py
  scripts/wind_turbine_scene.py
  scripts/xacro_strip_comments.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
