#include "input4r2_command_sender/grsim_sender.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<input4r2_command_sender::GrsimSender>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}