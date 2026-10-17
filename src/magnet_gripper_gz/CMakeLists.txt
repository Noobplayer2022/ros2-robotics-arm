cmake_minimum_required(VERSION 3.10)
project(magnet_gripper_gz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# --- روش استاندارد ROS2: vendorها ---
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS all)

find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)

add_library(MagnetGripperSystem SHARED
  src/MagnetGripperSystem.cc
)

target_include_directories(MagnetGripperSystem
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# نام targetها بین نسخه‌ها فرق می‌کند؛ این حالت معمولاً کار می‌کند:
target_link_libraries(MagnetGripperSystem
  PUBLIC
    gz-sim::gz-sim
    gz-plugin::gz-plugin
    gz-transport::gz-transport
    gz-msgs::gz-msgs
    gz-math::gz-math
)

install(
  TARGETS MagnetGripperSystem
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()