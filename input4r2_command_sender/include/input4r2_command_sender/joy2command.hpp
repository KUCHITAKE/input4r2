#ifndef INPUT4R2_COMMAND_SENDER__JOY2COMMAND_HPP_
#define INPUT4R2_COMMAND_SENDER__JOY2COMMAND_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "input4r2_interface/msg/command.hpp"

namespace input4r2_command_sender {
class Joy2Command : public rclcpp::Node {
 public:
  Joy2Command(const rclcpp::NodeOptions& options);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  rclcpp::Publisher<input4r2_interface::msg::Command>::SharedPtr
      command_publisher;
};
}  // namespace input4r2_command_sender

#endif  // INPUT4R2_COMMAND_SENDER__JOY2COMMAND_HPP_