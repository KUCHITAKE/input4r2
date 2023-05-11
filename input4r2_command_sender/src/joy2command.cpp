#include "input4r2_command_sender/joy2command.hpp"

namespace input4r2_command_sender {
Joy2Command::Joy2Command(const rclcpp::NodeOptions& options)
    : Node("joy2command", options) {
  declare_parameter("is_team_yellow", true);
  declare_parameter("id", 0);

  joy_subscription = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&Joy2Command::joy_callback, this, std::placeholders::_1));
  command_publisher =
      create_publisher<input4r2_interface::msg::Command>("command", 10);
}

void Joy2Command::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  input4r2_interface::msg::Command command;
  command.is_team_yellow = get_parameter("is_team_yellow").as_bool();

  input4r2_interface::msg::RobotCommand robot_command;
  robot_command.id = get_parameter("id").as_int();
  robot_command.cmd_vel.linear.x = msg->axes[1];
  robot_command.cmd_vel.linear.y = msg->axes[0];
  robot_command.cmd_vel.angular.z = msg->axes[3] * M_PI;
  robot_command.kick_speed_x = msg->buttons[1] * 3.0;
  robot_command.kick_speed_z = msg->axes[0] * 3.0;
  robot_command.dribble_speed = 1.0 - msg->axes[5];

  command.robot_commands.push_back(robot_command);

  command_publisher->publish(command);
}
}  // namespace input4r2_command_sender

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(input4r2_command_sender::Joy2Command)