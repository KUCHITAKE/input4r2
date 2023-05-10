#include "input4r2_vision/vision.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<input4r2_vision::Vision>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}