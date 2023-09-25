#include "rclcpp/rclcpp.hpp"
#include "UDPSocket.hpp"
#include "communication_interfaces/msg/game_control_data.hpp"

class GameControllerReceiver : public rclcpp::Node
{
public:
  GameControllerReceiver()
  : Node("game_controller_receiver")
  {
    socket_ = new UDPSocket(port_);
    publisher_ = this->create_publisher<communication_interfaces::msg::GameControlData>(
      "~/gc_data",
      10);
  }

  ~GameControllerReceiver()
  {
    delete socket_;
  }

  void read_and_publish()
  {
    int const buf_size = 1024;
    uint8_t buf[buf_size];

    auto const len = socket_->read_socket(buf, buf_size);
    if (len == -1) {
      RCLCPP_WARN(this->get_logger(), "Failed to read socket.");
      return;
    } else if (len == buf_size) {
      RCLCPP_WARN(this->get_logger(), "Buffer may be full.");
      return;
    }

    communication_interfaces::msg::GameControlData data =
      readGameControlData(buf);

    publisher_->publish(data);
  }

private:
  int const port_ = 3838;
  UDPSocket * socket_;
  rclcpp::Publisher<communication_interfaces::msg::GameControlData>::SharedPtr publisher_;

  communication_interfaces::msg::GameControlData readGameControlData(uint8_t const * buf)
  {
    // size of buf must be 688 (RGme 12)

    communication_interfaces::msg::GameControlData msg;

    msg.std_header.stamp = this->get_clock()->now();
    msg.header += (char)buf[0];
    msg.header += (char)buf[1];
    msg.header += (char)buf[2];
    msg.header += (char)buf[3];
    msg.protocol_version = (uint16_t)(buf[5] << 8 | buf[4]);
    msg.packet_number = buf[6];
    msg.players_per_team = buf[7];
    msg.game_type = buf[8];
    msg.state = buf[9];
    msg.first_half = buf[10];
    msg.kick_off_team = buf[11];
    msg.secondary_state = buf[12];
    msg.secondary_state_info.at(0) = buf[13];
    msg.secondary_state_info.at(1) = buf[14];
    msg.secondary_state_info.at(2) = buf[15];
    msg.secondary_state_info.at(3) = buf[16];
    msg.drop_in_team = buf[17];
    msg.drop_in_time = (uint16_t)(buf[19] << 8 | buf[18]);
    msg.secs_remaining = (uint16_t)(buf[21] << 8 | buf[20]);
    msg.secondary_time = (uint16_t)(buf[23] << 8 | buf[22]);
    msg.teams.at(0) = readTeamInfo(buf + 24);
    msg.teams.at(1) = readTeamInfo(buf + 356);

    return msg;
  }

  communication_interfaces::msg::TeamInfo readTeamInfo(uint8_t const * buf)
  {
    communication_interfaces::msg::TeamInfo msg;

    msg.team_number = buf[0];
    msg.team_colour = buf[1];
    msg.score = buf[2];
    msg.penalty_shot = buf[3];
    msg.single_shots = (uint16_t)(buf[5] << 8 | buf[4]);
    msg.coach_sequence = buf[6];
    for (int i = 0; i < 253; ++i) {
      msg.coach_message.at(i) = buf[7 + i];
    }
    msg.coach = readRobotInfo(buf + 260);
    for (int i = 0; i < 11; ++i) {
      msg.players.at(i) = readRobotInfo(buf + 266 + 6 * i);
    }

    return msg;
  }

  communication_interfaces::msg::RobotInfo readRobotInfo(uint8_t const * buf)
  {
    communication_interfaces::msg::RobotInfo msg;

    msg.penalty = buf[0];
    msg.secs_till_unpenalised = buf[1];
    msg.warning_count = buf[2];
    msg.yellow_card_count = buf[3];
    msg.red_card_count = buf[4];
    msg.goalkeeper = (bool)buf[5];

    return msg;
  }
};
