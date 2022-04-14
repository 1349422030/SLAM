#ifndef CAM_NODE_HPP
#define CAM_NODE_HPP

#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace std;

class CamNode : public rclcpp::Node {
public:
  CamNode();

  // void StereoDepth(int *image_count, string *left_image_list,
  //                  string *right_image_list);

private:
  void SysInit();
  int GetImageCount(string dir);
  void GetImages(string dir, int *image_count, string *image_list);
  void ImagePub();

  string left_image_path_;
  string right_image_path_;
};

#endif
