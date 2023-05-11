cmake_minimum_required(VERSION 3.8)
project(input4r2_interface)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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

  "msg/gc/referee/MatchType.msg"
  "msg/gc/referee/RefereeCommand.msg"
  "msg/gc/referee/RefereeStage.msg"
  "msg/gc/referee/RefereeTeamInfo.msg"
  "msg/gc/referee/Referee.msg"

  "msg/comm/RobotCommand.msg"
  "msg/comm/Command.msg"

  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
