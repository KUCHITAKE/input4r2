cmake_minimum_required(VERSION 3.8)
project(input4r2_gc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_subdirectory(proto)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(input4r2_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

add_library(gc SHARED
  src/gc.cpp
)
target_include_directories(gc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_nodes(gc "input4r2_gc::Gc")
ament_target_dependencies(gc
  rclcpp
  rclcpp_components
  input4r2_interface
)
target_link_libraries(gc
  Boost::system
  proto
)

add_executable(gc_node
  src/gc_node.cpp
)
ament_target_dependencies(gc_node
  rclcpp
)
target_link_libraries(gc_node
  gc
)

install(TARGETS
  gc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gc_node
  DESTINATION lib/${PROJECT_NAME}
) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
