cmake_minimum_required(VERSION 3.8)
project(chroma_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models rviz scripts urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/jethexa_gait.py scripts/zone_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
