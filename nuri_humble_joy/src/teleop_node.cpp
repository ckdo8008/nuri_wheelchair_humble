#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "nuri_humble_joy/teleop_joy.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::spin(std::make_unique<TeleopJoy>(rclcpp::NodeOptions()));
  // rclcpp::shutdown();

    auto node = std::make_shared<TeleopJoy>(rclcpp::NodeOptions()); // What is auto??
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();      

  return 0;
}