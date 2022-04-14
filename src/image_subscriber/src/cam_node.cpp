#include <filesystem>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "image_subscriber/cam_node.hpp"
#include "image_subscriber/optical_flow.hpp"

namespace fs = std::filesystem;

CamNode::CamNode() : Node("parameter_node") {
  SysInit();

  int image_count = GetImageCount(left_image_path_);
  string *left_image_list = new string[image_count];
  string *right_image_list = new string[image_count];
  GetImages(left_image_path_, &image_count, left_image_list);
  GetImages(right_image_path_, &image_count, right_image_list);

  OpticalFlowTracker::SingleLKFlow(&image_count, left_image_list);
}

void CamNode::SysInit() {
  // Get OpenCV Version
  RCLCPP_INFO(this->get_logger(), "\nOpenCV Version = %s", CV_VERSION);

  // Get Image Path
  string image_base_path;
  string left_sub_image_path;
  string right_sub_image_path;

  this->declare_parameter<string>("image_base_path", "");
  this->get_parameter("image_base_path", image_base_path);
  // RCLCPP_INFO(this->get_logger(), "\nImage Base Path = %s",
  //             image_base_path.c_str());

  this->declare_parameter<string>("left_sub_image_path", "");
  this->get_parameter("left_sub_image_path", left_sub_image_path);
  this->declare_parameter<string>("right_sub_image_path", "");
  this->get_parameter("right_sub_image_path", right_sub_image_path);
  // RCLCPP_INFO(this->get_logger(), "\nLeft Image Sub Path = %s",
  //             left_sub_image_path.c_str());
  // RCLCPP_INFO(this->get_logger(), "\nRight Image Sub Path = %s",
  //             right_sub_image_path.c_str());

  left_image_path_ = image_base_path + left_sub_image_path;
  right_image_path_ = image_base_path + right_sub_image_path;
  // RCLCPP_INFO(this->get_logger(), "\nLeft Image Path = %s",
  //             left_image_path_.c_str());
  // RCLCPP_INFO(this->get_logger(), "\nRight Image Path = %s",
  //             right_image_path_.c_str());
}

int CamNode::GetImageCount(string dir) {
  using fp = bool (*)(const std::filesystem::path &);
  return count_if(fs::directory_iterator(dir), fs::directory_iterator{},
                  (fp)fs::is_regular_file);
}

void CamNode::GetImages(string dir, int *image_count, string *image_list) {
  int file_count = 0;
  for (auto const &dir_entry : fs::directory_iterator{dir}) {
    // cout << dir_entry << '\n';
    if (file_count < *image_count)
      *(image_list + file_count++) = dir_entry.path();
  }
}

// void CamNode::StereoDepth(int *image_count, string *left_image_list,
//                           string *right_image_list) {}
