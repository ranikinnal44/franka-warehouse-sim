cmake_minimum_required(VERSION 3.5)
project(franka_warehouse_world)

find_package(ament_cmake REQUIRED)

# Install launch files, worlds, config and scripts.
install(
  DIRECTORY
    launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_world.py
    scripts/publish_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
