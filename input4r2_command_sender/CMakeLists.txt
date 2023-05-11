cmake_minimum_required(VERSION 3.8)
project(input4r2_command_sender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(proto)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(input4r2_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

add_library(grsim_sender SHARED
  src/grsim_sender.cpp
  include/input4r2_command_sender/grsim_sender.hpp
)
target_include_directories(grsim_sender PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_nodes(grsim_sender "input4r2_command_sender::GrsimSender")
ament_target_dependencies(grsim_sender
  rclcpp
  rclcpp_components
  geometry_msgs
  input4r2_interface
)
target_link_libraries(grsim_sender
  Boost::system
  proto
)

add_executable(grsim_sender_node
  src/grsim_sender_node.cpp
)
ament_target_dependencies(grsim_sender_node
  rclcpp
)
target_link_libraries(grsim_sender_node
  grsim_sender
)

install(TARGETS
  grsim_sender
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  grsim_sender_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
