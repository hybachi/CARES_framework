cmake_minimum_required(VERSION 3.8)
project(chroma_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-array-bounds -Wno-stringop-overflow -Wno-deprecated-declarations)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(chroma_interfaces REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_SKIP_RPATH TRUE)
set(CMAKE_SKIP_INSTALL_RPATH TRUE)

add_library(chroma_rviz_plugins SHARED
  src/mission_panel.cpp
  include/chroma_rviz_plugins/mission_panel.hpp
  src/status_panel.cpp                              
  include/chroma_rviz_plugins/status_panel.hpp
  src/task_panel.cpp                             
  include/chroma_rviz_plugins/task_panel.hpp
  src/log_panel.cpp                         
  include/chroma_rviz_plugins/log_panel.hpp 
  src/fault_panel.cpp
  include/chroma_rviz_plugins/fault_panel.hpp
)

target_include_directories(chroma_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(chroma_rviz_plugins
  rclcpp
  rviz_common
  pluginlib
  geometry_msgs
  visualization_msgs
  chroma_interfaces
)

target_link_libraries(chroma_rviz_plugins Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

install(TARGETS chroma_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rclcpp rviz_common pluginlib Qt5Widgets geometry_msgs chroma_interfaces)
ament_package()
