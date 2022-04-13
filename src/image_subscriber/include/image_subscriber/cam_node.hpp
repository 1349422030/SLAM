#ifndef CAM_NODE_HPP
#define CAM_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

class CamNode : public rclcpp::Node {
public:
  CamNode();
  void GetImages(std::string dir, int *file_count, std::string *file_list);

private:
  void SysInit();

  std::string left_image_path_;
  std::string right_image_path_;
};

#endif
