#include "input4r2_gc/gc.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<input4r2_gc::Gc>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}