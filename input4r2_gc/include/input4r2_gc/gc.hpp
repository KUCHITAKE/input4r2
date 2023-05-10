#ifndef INPUT4R2_GC__GC_HPP_
#define INPUT4R2_GC__GC_HPP_

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>

#include "input4r2_interface/msg/referee.hpp"
#include "ssl_gc_api.pb.h"

namespace input4r2_gc {

using boost::asio::ip::udp;

class Gc : public rclcpp::Node {
 public:
  explicit Gc(const rclcpp::NodeOptions& options);

 private:
  void do_receive();

  input4r2_interface::msg::Referee convert_referee(const Referee& referee);

  boost::asio::io_context io_context;
  udp::socket socket;
  udp::endpoint sender_endpoint;
  std::array<char, 4096> recv_buffer;
  std::thread io_thread;

  rclcpp::Publisher<input4r2_interface::msg::Referee>::SharedPtr referee_publisher;
};
}  // namespace input4r2_gc

#endif  // INPUT4R2_GC__GC_HPP_
