#ifndef CAM_NODE_HPP
#define CAM_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

struct CAMERA_INTRINSIC_PARAMETERS {
  double fx, fy, cx, cy, k1, k2, p1, p2;
  double T[4][4];
};

class CamNode : public rclcpp::Node {
public:
  CamNode();

  void StereoDepth(int *image_count, string *left_image_list,
                   string *right_image_list);

  void undistort(cv::Mat &left_image, cv::Mat &right_image,
                 CAMERA_INTRINSIC_PARAMETERS left_camera,
                 CAMERA_INTRINSIC_PARAMETERS right_camera,
                 Eigen::Matrix4d T_BS);

  void stereoSGBM(cv::Mat lpng, cv::Mat rpng, cv::Mat &disp);

  void disp2Depth(cv::Mat disp, cv::Mat &depth,
                  CAMERA_INTRINSIC_PARAMETERS camera);

  void showPointCloud(
      const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

private:
  void SysInit();
  int GetImageCount(string dir);
  void GetImages(string dir, int &image_count, string *image_list);
  void ImagePub();

  string left_image_path_;
  string right_image_path_;
};

#endif
