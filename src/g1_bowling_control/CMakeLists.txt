cmake_minimum_required(VERSION 3.8)
project(g1_bowling_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python library (g1_bowling_control.*). Scripts are installed as programs with
# an "env python3" shebang so they run in the active venv (mujoco, torch).
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/walk_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch policies
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
