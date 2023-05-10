cmake_minimum_required(VERSION 3.8)
project(input4r2_interface)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/vision/detection/DetectionBall.msg"
  "msg/vision/detection/DetectionRobot.msg"
  "msg/vision/detection/Detection.msg"
  "msg/vision/geometry/FieldShapeType.msg"
  "msg/vision/geometry/FieldLineSegment.msg"
  "msg/vision/geometry/FieldCircularArc.msg"
  "msg/vision/geometry/GeometryCameraCalibration.msg"
  "msg/vision/geometry/GeometryFieldSize.msg"
  "msg/vision/geometry/GeometryData.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
