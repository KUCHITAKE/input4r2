#include "input4r2_command_sender/grsim_sender.hpp"

#include "grSim_Packet.pb.h"

namespace input4r2_command_sender {

GrsimSender::GrsimSender(const rclcpp::NodeOptions& options)
    : Node("grsim_sender", options), socket(io_service) {
  declare_parameter("grsim_address", "127.0.0.1");
  declare_parameter("grsim_port", 20011);

  command_subscription =
      this->create_subscription<input4r2_interface::msg::Command>(
          "command", 1,
          std::bind(&GrsimSender::command_callback, this,
                    std::placeholders::_1));

  endpoint = boost::asio::ip::udp::endpoint(
      boost::asio::ip::address::from_string(
          get_parameter("grsim_address").as_string()),
      get_parameter("grsim_port").as_int());

  socket.open(boost::asio::ip::udp::v4());
}

void GrsimSender::command_callback(
    const input4r2_interface::msg::Command::SharedPtr msg) {
  grSim_Packet packet;

  auto* commands = packet.mutable_commands();
  commands->set_timestamp(0.0);
  commands->set_isteamyellow(msg->is_team_yellow);
  for (const auto& robot_command_msg : msg->robot_commands) {
    grSim_Robot_Command* robot_command = commands->add_robot_commands();
    robot_command->set_id(robot_command_msg.id);
    robot_command->set_wheelsspeed(false);
    robot_command->set_veltangent(robot_command_msg.cmd_vel.linear.x);
    robot_command->set_velnormal(robot_command_msg.cmd_vel.linear.y);
    robot_command->set_velangular(robot_command_msg.cmd_vel.angular.z);
    robot_command->set_kickspeedx(robot_command_msg.kick_speed_x);
    robot_command->set_kickspeedz(robot_command_msg.kick_speed_z);
    robot_command->set_spinner(robot_command_msg.dribble_speed > 0.0);
  }

  socket.send_to(boost::asio::buffer(packet.SerializeAsString()), endpoint);
}
}  // namespace input4r2_command_sender

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(input4r2_command_sender::GrsimSender)