#ifndef INPUT4R2_VISION__VISION_HPP_
#define INPUT4R2_VISION__VISION_HPP_

#include <boost/asio.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <input4r2_interface/msg/robot_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace input4r2_command_sender {

struct RobotCommandSubscriber {
  rclcpp::Subscription<input4r2_interface::msg::RobotCommand>::SharedPtr
      robot_command_subscription;

  void robot_command_callback(
      const input4r2_interface::msg::RobotCommand::SharedPtr msg);
};

class GrsimSender : public rclcpp::Node {
 public:
  GrsimSender(const rclcpp::NodeOptions& options);

 private:
  std::array<RobotCommandSubscriber, 16> robot_command_subscribers;
  // boost::asio::io_service io_service;
  // boost::asio::ip::udp::socket socket;
  // boost::asio::ip::udp::endpoint endpoint;
};
}  // namespace input4r2_command_sender

#endif  // INPUT4R2_VISION__VISION_HPP_
