#ifndef INPUT4R2_VISION__VISION_HPP_
#define INPUT4R2_VISION__VISION_HPP_

#include <boost/asio.hpp>

#include "input4r2_interface/msg/detection.hpp"
#include "input4r2_interface/msg/geometry_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ssl_vision_wrapper.pb.h"

namespace input4r2_vision {

using boost::asio::ip::udp;

//! @class Vision
//! @brief Vision is a class that extends rclcpp::Node and receives SSL-Vision
//! packets.
//!
//! Vision class listens for SSL-Vision packets and publishes the received data
//! as input4r2_interface::msg::Detection and
//! input4r2_interface::msg::GeometryData messages. The node has the following
//! parameters:
//! @b "multicast_address": The multicast address to join for receiving
//! SSL-Vision packets (default: "224.5.23.2").
//! @b "multicast_port": The multicast port for receiving SSL-Vision packets
//! (default: 10006).
//!
//! The Vision class has two publishers:
//! @b "~/detection": A publisher for input4r2_interface::msg::Detection
//! messages with a default QoS of 1.
//! @b "~/geometry": A publisher for input4r2_interface::msg::GeometryData
//! messages with a default QoS of 1.
class Vision : public rclcpp::Node {
 public:
  //! \brief Constructs a Vision object.
  //!
  //! \param options rclcpp::NodeOptions for the node.
  explicit Vision(const rclcpp::NodeOptions& options);

 private:
  //! \brief Initiates an asynchronous receive operation.
  void do_receive();

  //! \brief Converts an SSL_DetectionFrame to
  //! input4r2_interface::msg::Detection.
  //!
  //! \param detection_frame The SSL_DetectionFrame to convert.
  //! \return The converted input4r2_interface::msg::Detection message.
  static input4r2_interface::msg::Detection convert_detection(
      const SSL_DetectionFrame& detection_frame);
  //! \brief Converts an SSL_GeometryData to
  //! input4r2_interface::msg::GeometryData.
  //!
  //! \param geometry_data The SSL_GeometryData to convert.
  //! \return The converted input4r2_interface::msg::GeometryData message.
  static input4r2_interface::msg::GeometryData convert_geometry(
      const SSL_GeometryData& geometry_data);

  rclcpp::Publisher<input4r2_interface::msg::Detection>::SharedPtr
      detection_publisher;
  rclcpp::Publisher<input4r2_interface::msg::GeometryData>::SharedPtr
      geometry_publisher;

  boost::asio::io_context io_context;
  udp::socket socket;
  udp::endpoint sender_endpoint;
  std::array<char, 4096> data;
  std::thread io_thread;
};

}  // namespace input4r2_vision

#endif  // INPUT4R2_VISION__VISION_HPP_