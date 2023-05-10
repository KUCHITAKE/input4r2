#include "input4r2_vision/vision.hpp"

#include <boost/bind/bind.hpp>

namespace input4r2_vision {

Vision::Vision(const rclcpp::NodeOptions& options)
    : Node("vision", options), socket(io_context) {
  declare_parameter("multicast_address", "224.5.23.2");
  declare_parameter("multicast_port", 10006);

  detection_publisher = create_publisher<input4r2_interface::msg::Detection>(
      "~/detection", rclcpp::QoS(1));

  geometry_publisher = create_publisher<input4r2_interface::msg::GeometryData>(
      "~/geometry", rclcpp::QoS(1));

  udp::endpoint listen_endpoint(boost::asio::ip::address_v4::any(),
                                get_parameter("multicast_port").as_int());
  socket.open(listen_endpoint.protocol());
  socket.set_option(udp::socket::reuse_address(true));
  socket.bind(listen_endpoint);

  socket.set_option(boost::asio::ip::multicast::join_group(
      boost::asio::ip::address::from_string(
          get_parameter("multicast_address").as_string())));

  do_receive();

  io_thread = std::thread([this]() { io_context.run(); });

  RCLCPP_INFO_STREAM(
      get_logger(), "Listening on "
                        << listen_endpoint.address() << ":"
                        << listen_endpoint.port() << "(multicast "
                        << get_parameter("multicast_address").as_string() << ":"
                        << get_parameter("multicast_port").as_int() << ")"
                        << " for vision packets");
}

void Vision::do_receive() {
  socket.async_receive_from(
      boost::asio::buffer(data), sender_endpoint,
      [this](boost::system::error_code ec, std::size_t length) {
        if (!ec) {
          SSL_WrapperPacket packet;
          packet.ParseFromArray(data.data(), length);
          if (packet.has_detection()) {
            detection_publisher->publish(convert_detection(packet.detection()));
          }
          if (packet.has_geometry()) {
            geometry_publisher->publish(convert_geometry(packet.geometry()));
          }
          do_receive();
        }
      });
}

input4r2_interface::msg::Detection Vision::convert_detection(
    const SSL_DetectionFrame& detection_frame) {
  input4r2_interface::msg::Detection detection;

  detection.camera_id = detection_frame.camera_id();
  detection.frame_number = detection_frame.frame_number();
  detection.t_capture = detection_frame.t_capture();
  detection.t_sent = detection_frame.t_sent();

  std::vector<input4r2_interface::msg::DetectionBall> balls;
  for (const auto& ball : detection_frame.balls()) {
    input4r2_interface::msg::DetectionBall detection_ball;
    detection_ball.confidence = ball.confidence();
    detection_ball.area = ball.area();
    detection_ball.x = ball.x();
    detection_ball.y = ball.y();
    detection_ball.z = ball.z();
    detection_ball.pixel_x = ball.pixel_x();
    detection_ball.pixel_y = ball.pixel_y();

    balls.push_back(detection_ball);
  }
  detection.balls = balls;

  std::vector<input4r2_interface::msg::DetectionRobot> robots_blue;
  for (const auto& robot : detection_frame.robots_blue()) {
    input4r2_interface::msg::DetectionRobot detection_robot;
    detection_robot.confidence = robot.confidence();
    detection_robot.robot_id = robot.robot_id();
    detection_robot.x = robot.x();
    detection_robot.y = robot.y();
    detection_robot.orientation = robot.orientation();
    detection_robot.pixel_x = robot.pixel_x();
    detection_robot.pixel_y = robot.pixel_y();

    robots_blue.push_back(detection_robot);
  }
  detection.robots_blue = robots_blue;

  std::vector<input4r2_interface::msg::DetectionRobot> robots_yellow;
  for (const auto& robot : detection_frame.robots_yellow()) {
    input4r2_interface::msg::DetectionRobot detection_robot;
    detection_robot.confidence = robot.confidence();
    detection_robot.robot_id = robot.robot_id();
    detection_robot.x = robot.x();
    detection_robot.y = robot.y();
    detection_robot.orientation = robot.orientation();
    detection_robot.pixel_x = robot.pixel_x();
    detection_robot.pixel_y = robot.pixel_y();

    robots_yellow.push_back(detection_robot);
  }
  detection.robots_yellow = robots_yellow;

  return detection;
}

input4r2_interface::msg::GeometryData Vision::convert_geometry(
    const SSL_GeometryData& geometry_data) {
  input4r2_interface::msg::GeometryData geometry;

  input4r2_interface::msg::GeometryFieldSize field_size;
  const auto& field = geometry_data.field();
  field_size.field_length = field.field_length();
  field_size.field_width = field.field_width();
  field_size.goal_width = field.goal_width();
  field_size.goal_depth = field.goal_depth();
  field_size.boundary_width = field.boundary_width();
  field_size.penalty_area_depth = field.penalty_area_depth();
  field_size.penalty_area_width = field.penalty_area_width();

  std::vector<input4r2_interface::msg::FieldLineSegment> field_line_segments;
  for (const auto& line : field.field_lines()) {
    input4r2_interface::msg::FieldLineSegment field_line_segment;
    field_line_segment.name = line.name();
    field_line_segment.x1 = line.p1().x();
    field_line_segment.y1 = line.p1().y();
    field_line_segment.x2 = line.p2().x();
    field_line_segment.y2 = line.p2().y();
    field_line_segment.thickness = line.thickness();

    field_line_segments.push_back(field_line_segment);
  }
  field_size.field_lines = field_line_segments;

  std::vector<input4r2_interface::msg::FieldCircularArc> field_circular_arcs;
  for (const auto& arc : field.field_arcs()) {
    input4r2_interface::msg::FieldCircularArc field_circular_arc;
    field_circular_arc.name = arc.name();
    field_circular_arc.center_x = arc.center().x();
    field_circular_arc.center_y = arc.center().y();
    field_circular_arc.radius = arc.radius();
    field_circular_arc.a1 = arc.a1();
    field_circular_arc.a2 = arc.a2();
    field_circular_arc.thickness = arc.thickness();

    field_circular_arcs.push_back(field_circular_arc);
  }
  field_size.field_arcs = field_circular_arcs;

  geometry.field = field_size;

  std::vector<input4r2_interface::msg::GeometryCameraCalibration>
      camera_calibrations;
  for (const auto& camera : geometry_data.calib()) {
    input4r2_interface::msg::GeometryCameraCalibration camera_calibration;
    camera_calibration.camera_id = camera.camera_id();
    camera_calibration.focal_length = camera.focal_length();
    camera_calibration.principal_point_x = camera.principal_point_x();
    camera_calibration.principal_point_y = camera.principal_point_y();
    camera_calibration.distortion = camera.distortion();
    camera_calibration.q0 = camera.q0();
    camera_calibration.q1 = camera.q1();
    camera_calibration.q2 = camera.q2();
    camera_calibration.q3 = camera.q3();
    camera_calibration.tx = camera.tx();
    camera_calibration.ty = camera.ty();
    camera_calibration.tz = camera.tz();
    camera_calibration.derived_camera_world_tx =
        camera.derived_camera_world_tx();
    camera_calibration.derived_camera_world_ty =
        camera.derived_camera_world_ty();
    camera_calibration.derived_camera_world_tz =
        camera.derived_camera_world_tz();
    camera_calibration.pixel_image_width = camera.pixel_image_width();
    camera_calibration.pixel_image_height = camera.pixel_image_height();

    camera_calibrations.push_back(camera_calibration);
  }
  geometry.calib = camera_calibrations;

  return geometry;
}

}  // namespace input4r2_vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(input4r2_vision::Vision)