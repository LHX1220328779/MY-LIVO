#include "LIVMapper.h"
#include <iostream>
#include <csignal>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }
}

int main(int argc, char **argv)
{
  rclcpp::Node::SharedPtr node;
  try
  {
    configureRosLogDirectory();
    // Finish the accepted scans and publish the final solution BEFORE ROS
    // shutdown. The default signal handler shuts publishers down too early.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    node = std::make_shared<rclcpp::Node>("laser_mapping");
    LIVMapper mapper(node);
    mapper.initializeSubscribersAndPublishers();
    mapper.run(stop_requested);
  }
  catch (const std::exception &error)
  {
    if (node) RCLCPP_FATAL(node->get_logger(), "Mapping failed: %s", error.what());
    else std::cerr << "Mapping failed: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
