#include "input4r2_command_sender/joy2command.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<input4r2_command_sender::Joy2Command>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}