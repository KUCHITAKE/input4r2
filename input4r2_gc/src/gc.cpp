#include "input4r2_gc/gc.hpp"

namespace input4r2_gc {
Gc::Gc(const rclcpp::NodeOptions& options)
    : Node("gc", options), socket(io_context) {
  declare_parameter("multicast_address", "224.5.23.1");
  declare_parameter("multicast_port", 10003);

  referee_publisher = create_publisher<input4r2_interface::msg::Referee>(
      "~/referee", rclcpp::QoS(1));

  udp::endpoint listen_endpoint(boost::asio::ip::address_v4::any(),
                                get_parameter("multicast_port").as_int());
  socket.open(listen_endpoint.protocol());
  socket.set_option(udp::socket::reuse_address(true));
  socket.bind(listen_endpoint);

  socket.set_option(boost::asio::ip::multicast::join_group(
      boost::asio::ip::address::from_string(
          get_parameter("multicast_address").as_string())));

  do_receive();

  io_thread = std::thread([this]() { io_context.run(); });

  RCLCPP_INFO_STREAM(
      get_logger(), "Listening on "
                        << socket.local_endpoint().address() << ":"
                        << socket.local_endpoint().port() << "(multicast "
                        << get_parameter("multicast_address").as_string() << ":"
                        << get_parameter("multicast_port").as_int() << ")");
}

void Gc::do_receive() {
  socket.async_receive_from(
      boost::asio::buffer(recv_buffer), sender_endpoint,
      [this](boost::system::error_code ec, std::size_t length) {
        if (!ec) {
          Referee referee;
          referee.ParseFromArray(recv_buffer.data(), length);
          referee_publisher->publish(convert_referee(referee));
        }

        do_receive();
      });
}

input4r2_interface::msg::Referee Gc::convert_referee(const Referee& referee) {
  input4r2_interface::msg::Referee msg;
  msg.packet_timestamp = referee.packet_timestamp();
  msg.stage = referee.stage();
  msg.command = referee.command();
  msg.command_counter = referee.command_counter();
  msg.command_timestamp = referee.command_timestamp();
  msg.stage_time_left = referee.stage_time_left();
  msg.designated_position_x = referee.designated_position().x();
  msg.designated_position_y = referee.designated_position().y();
  msg.blue_team_on_positive_half = referee.blue_team_on_positive_half();
  msg.next_command = referee.next_command();
  msg.current_action_time_remaining = referee.current_action_time_remaining();
  msg.source_identifier = referee.source_identifier();
  msg.match_type = referee.match_type();

  input4r2_interface::msg::RefereeTeamInfo teaminfo_yellow_msg;
  const auto& yelow = referee.yellow();
  teaminfo_yellow_msg.name = yelow.name();
  teaminfo_yellow_msg.score = yelow.score();
  teaminfo_yellow_msg.red_cards = yelow.red_cards();
  teaminfo_yellow_msg.yellow_cards = yelow.yellow_cards();
  teaminfo_yellow_msg.timeouts = yelow.timeouts();
  teaminfo_yellow_msg.timeout_time = yelow.timeout_time();
  teaminfo_yellow_msg.goalkeeper = yelow.goalkeeper();
  teaminfo_yellow_msg.foul_counter = yelow.foul_counter();
  teaminfo_yellow_msg.ball_placement_failures = yelow.ball_placement_failures();
  teaminfo_yellow_msg.can_place_ball = yelow.can_place_ball();
  teaminfo_yellow_msg.max_allowed_bots = yelow.max_allowed_bots();
  teaminfo_yellow_msg.bot_substitution_intent = yelow.bot_substitution_intent();
  teaminfo_yellow_msg.ball_placement_failures_reached =
      yelow.ball_placement_failures_reached();
  teaminfo_yellow_msg.bot_substitution_allowed =
      yelow.bot_substitution_allowed();

  msg.yellow = teaminfo_yellow_msg;

  input4r2_interface::msg::RefereeTeamInfo teaminfo_blue_msg;
  const auto& blue = referee.blue();
  teaminfo_blue_msg.name = blue.name();
  teaminfo_blue_msg.score = blue.score();
  teaminfo_blue_msg.red_cards = blue.red_cards();
  teaminfo_blue_msg.yellow_cards = blue.yellow_cards();
  teaminfo_blue_msg.timeouts = blue.timeouts();
  teaminfo_blue_msg.timeout_time = blue.timeout_time();
  teaminfo_blue_msg.goalkeeper = blue.goalkeeper();
  teaminfo_blue_msg.foul_counter = blue.foul_counter();
  teaminfo_blue_msg.ball_placement_failures = blue.ball_placement_failures();
  teaminfo_blue_msg.can_place_ball = blue.can_place_ball();
  teaminfo_blue_msg.max_allowed_bots = blue.max_allowed_bots();
  teaminfo_blue_msg.bot_substitution_intent = blue.bot_substitution_intent();
  teaminfo_blue_msg.ball_placement_failures_reached =
      blue.ball_placement_failures_reached();
  teaminfo_blue_msg.bot_substitution_allowed = blue.bot_substitution_allowed();

  msg.blue = teaminfo_blue_msg;

  return msg;
}

}  // namespace input4r2_gc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(input4r2_gc::Gc)