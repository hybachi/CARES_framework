cmake_minimum_required(VERSION 3.8)
<<<<<<<< HEAD:src/chroma_bringup/CMakeLists.txt
project(chroma_bringup)
========
project(chroma_simulation)
>>>>>>>> 0e3e22a (jethexa simulation):src/chroma_simulation/CMakeLists.txt

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
<<<<<<<< HEAD:src/chroma_bringup/CMakeLists.txt
  DIRECTORY config launch robot
========
  DIRECTORY config launch models rviz scripts urdf worlds
>>>>>>>> 0e3e22a (jethexa simulation):src/chroma_simulation/CMakeLists.txt
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/jethexa_gait.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
