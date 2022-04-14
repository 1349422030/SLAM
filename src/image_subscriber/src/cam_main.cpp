#include "image_subscriber/cam_node.hpp"
#include "image_subscriber/optical_flow.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  // Init ROS
  rclcpp::init(argc, argv);

  // Create single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create and add camera node
  auto cam_node = std::make_shared<CamNode>();
  executor.add_node(cam_node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  // Shut down ROS
  rclcpp::shutdown();
  return 0;
}
