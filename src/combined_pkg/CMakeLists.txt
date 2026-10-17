cmake_minimum_required(VERSION 3.8)
project(combined_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# نصب فایل‌ها
install(DIRECTORY
  urdf
  meshes
  launch
  config
  scripts
  models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller_base.py
  scripts/pso_optimizer.py
  scripts/pid_controller_node.py
  scripts/smc_controller_node.py
  scripts/mpc_controller_node.py
  scripts/control_panel.py
  scripts/main.py
  scripts/cube_spawner.py
  scripts/magnet_gripper_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()