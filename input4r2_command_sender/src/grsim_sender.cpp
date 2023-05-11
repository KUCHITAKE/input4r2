#include "input4r2_command_sender/grsim_sender.hpp"

#include "grSim_Packet.pb.h"

namespace input4r2_command_sender {

void RobotCommandSubscriber::robot_command_callback(
    const input4r2_interface::msg::RobotCommand::SharedPtr msg) {
  grSim_Packet packet;
  auto* packet_commands = packet.mutable_commands();
  packet_commands->set_timestamp(0.0);
}

GrsimSender::GrsimSender(const rclcpp::NodeOptions& options)
    : Node("grsim_sender", options) {
  for (auto i = 0; i < 16; ++i) {
    robot_command_subscribers[i].robot_command_subscription =
        create_subscription<input4r2_interface::msg::RobotCommand>(
            "~/robot" + std::to_string(i) + "/robot_command", 1,
            std::bind(&RobotCommandSubscriber::robot_command_callback,
                      &robot_command_subscribers[i], std::placeholders::_1));
  }
}
}  // namespace input4r2_command_sender

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(input4r2_command_sender::GrsimSender)