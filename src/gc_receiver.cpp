#include "gc_receiver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GameControllerReceiver>();
  while (rclcpp::ok()) {
    node->read_and_publish();
  }

  rclcpp::shutdown();

  return 0;
}
