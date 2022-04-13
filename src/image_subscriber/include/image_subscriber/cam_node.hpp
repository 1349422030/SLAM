#ifndef CAM_NODE_HPP
#define CAM_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <iostream>
#include <typeinfo>

namespace fs = std::filesystem;

class CamNode : public rclcpp::Node {
public:
  CamNode();
  std::string* GetImages(std::string dir);

private:
  void SysInit();

  std::string image_left_path_;
  std::string image_right_path_;
};

#endif