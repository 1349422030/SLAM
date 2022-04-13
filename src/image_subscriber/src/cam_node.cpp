#include "image_subscriber/cam_node.hpp"

CamNode::CamNode() : Node("parameter_node") {
  SysInit();
  std::string *left_images = GetImages(image_left_path_);
  RCLCPP_INFO(this->get_logger(), "\nImages Name = %s", &left_images);
}

void CamNode::SysInit() {
  // Get OpenCV Version
  RCLCPP_INFO(this->get_logger(), "\nOpenCV Version = %s", CV_VERSION);

  // Get Image Path
  std::string image_base_path;
  std::string image_left_sub_path;
  std::string image_right_sub_path;

  this->declare_parameter<std::string>("image_base_path", "");
  this->get_parameter("image_base_path", image_base_path);
  RCLCPP_INFO(this->get_logger(), "\nImage Path = %s", image_base_path.c_str());

  this->declare_parameter<std::string>("image_left_sub_path", "");
  this->get_parameter("image_left_sub_path", image_left_sub_path);
  this->declare_parameter<std::string>("image_right_sub_path", "");
  this->get_parameter("image_right_sub_path", image_right_sub_path);
  RCLCPP_INFO(this->get_logger(), "\nLeft Image Sub Path = %s",
              image_left_sub_path.c_str());
  RCLCPP_INFO(this->get_logger(), "\nRight Image Sub Path = %s",
              image_right_sub_path.c_str());

  image_left_path_ = image_base_path + image_left_sub_path;
  image_right_path_ = image_base_path + image_right_sub_path;
  RCLCPP_INFO(this->get_logger(), "\nLeft Image Path = %s",
              image_left_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "\nRight Image Path = %s",
              image_right_path_.c_str());
}

std::string *CamNode::GetImages(std::string dir) {
  std::string *images = new std::string[1000];
  int image_count = 0;

  for (const auto &entry : fs::directory_iterator(dir)) {
    RCLCPP_INFO(this->get_logger(), "\nImage Name = %s", entry.path().c_str());
    *(&images + image_count) = entry.path();
    if (image_count < 5 - 1) {
      *images = image_count;
      RCLCPP_INFO(this->get_logger(), "\nImage Count = %s", *images);
      break;
    }
  }
  return images;
}
