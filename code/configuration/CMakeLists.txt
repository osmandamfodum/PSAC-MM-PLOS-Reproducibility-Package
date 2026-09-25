cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch meshes worlds config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/drone_commander.py
    scripts/auto_drive.py
    scripts/mission_manager.py
    scripts/uav_detector_sim.py
    scripts/uav_world_tree_scanner.py
    scripts/uav_farm_scanner.py
    scripts/dynamic_obstacle_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/agri_mission
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
