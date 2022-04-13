#ifndef CAM_NODE_HPP
#define CAM_NODE_HPP

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Dense>
using namespace std;

class CamNode : public rclcpp::Node {
public:
  CamNode();
  void GetImages(string dir, int *file_count, string *file_list);
  void ImagePub();

  void SingleLKFlow(int *image_count, string *left_image_list,
                    string *right_image_list);
  void StereoDepth(int *image_count, string *left_image_list,
                          string *right_image_list);

private:
  void SysInit();

  string left_image_path_;
  string right_image_path_;
};

#endif
