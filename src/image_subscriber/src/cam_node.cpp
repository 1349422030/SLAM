#include "image_subscriber/cam_node.hpp"

CamNode::CamNode() : Node("parameter_node") {
  SysInit();

  std::string *left_image_list = new std::string[10000];
  std::string *right_image_list = new std::string[10000];
  int left_image_count = 0;
  int right_image_count = 0;
  GetImages(left_image_path_, &left_image_count, left_image_list);
  GetImages(left_image_path_, &right_image_count, right_image_list);




}

void CamNode::SysInit() {
  // Get OpenCV Version
  RCLCPP_INFO(this->get_logger(), "\nOpenCV Version = %s", CV_VERSION);

  // Get Image Path
  std::string image_base_path;
  std::string left_sub_image_path;
  std::string right_sub_image_path;

  this->declare_parameter<std::string>("image_base_path", "");
  this->get_parameter("image_base_path", image_base_path);
  RCLCPP_INFO(this->get_logger(), "\nImage Base Path = %s",
              image_base_path.c_str());

  this->declare_parameter<std::string>("left_sub_image_path", "");
  this->get_parameter("left_sub_image_path", left_sub_image_path);
  this->declare_parameter<std::string>("right_sub_image_path", "");
  this->get_parameter("right_sub_image_path", right_sub_image_path);
  RCLCPP_INFO(this->get_logger(), "\nLeft Image Sub Path = %s",
              left_sub_image_path.c_str());
  RCLCPP_INFO(this->get_logger(), "\nRight Image Sub Path = %s",
              right_sub_image_path.c_str());

  left_image_path_ = image_base_path + left_sub_image_path;
  right_image_path_ = image_base_path + right_sub_image_path;
  RCLCPP_INFO(this->get_logger(), "\nLeft Image Path = %s",
              left_image_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "\nRight Image Path = %s",
              right_image_path_.c_str());
}

void CamNode::GetImages(std::string dir, int *file_count,
                        std::string *file_list) {
  for (auto const &dir_entry : std::filesystem::directory_iterator{dir}) {
    // std::cout << dir_entry << '\n';
    *(file_list + (*file_count)++) = dir_entry.path();
  }
}
