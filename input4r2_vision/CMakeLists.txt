cmake_minimum_required(VERSION 3.8)
project(input4r2_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(proto)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(input4r2_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

add_library(vision SHARED
  src/vision.cpp
  include/input4r2_vision/vision.hpp
)
target_include_directories(vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_nodes(vision "input4r2_vision::Vision")
ament_target_dependencies(vision
  rclcpp
  rclcpp_components
  input4r2_interface
)
target_link_libraries(vision
  Boost::system
  proto
)

add_executable(vision_node
  src/vision_node.cpp
)
ament_target_dependencies(vision_node
  rclcpp
)
target_link_libraries(vision_node
  vision
)

install(TARGETS
  vision
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
