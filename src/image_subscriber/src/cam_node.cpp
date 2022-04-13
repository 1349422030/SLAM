#include <filesystem>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "image_subscriber/cam_node.hpp"

using namespace std;
namespace fs = std::filesystem;

CamNode::CamNode() : Node("parameter_node") {
  SysInit();

  string *left_image_list = new string[10000];
  string *right_image_list = new string[10000];
  int image_count = 0;
  GetImages(left_image_path_, &image_count, left_image_list);
  image_count = 0;
  GetImages(right_image_path_, &image_count, right_image_list);

  SingleLKFlow(&image_count, left_image_list, right_image_list);
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

void CamNode::GetImages(string dir, int *file_count, string *file_list) {
  for (auto const &dir_entry : filesystem::directory_iterator{dir})
    // cout << dir_entry << '\n';
    *(file_list + (*file_count)++) = dir_entry.path();
}

void CamNode::SingleLKFlow(int *image_count, string *left_image_list,
                     string *right_image_list) {
  list<cv::Point2f> keypoints; // 因为要删除跟踪失败的点，使用list
  cv::Mat image, last_image;

  for (int index = 100; index < *image_count; index++) {
    // 读入图像
    image = cv::imread(*(left_image_list + index), cv::IMREAD_COLOR);

    if (keypoints.size() < 100) {
      // 当特征点过少时重新提取特征点
      vector<cv::KeyPoint> kps;
      cv::Ptr<cv::FastFeatureDetector> detector =
          cv::FastFeatureDetector::create();
      detector->detect(image, kps);

      //记录所有特征点
      for (auto kp : kps)
        keypoints.push_back(kp.pt);
      last_image = image;
      continue;
    }

    vector<cv::Point2f> next_keypoints;
    vector<cv::Point2f> prev_keypoints;
    for (auto kp : keypoints)
      prev_keypoints.push_back(kp);
    vector<unsigned char> status;

    vector<float> error;

    //调用OpenCV官方LK算法并计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(last_image, image, prev_keypoints, next_keypoints,
                             status, error);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used =
        chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "LK Flow use time：" << time_used.count() << " seconds." << endl;

    // 把跟丢的点删掉
    int i = 0;
    for (auto iter = keypoints.begin(); iter != keypoints.end(); i++) {
      if (status[i] == 0) {
        iter = keypoints.erase(iter);
        continue;
      }
      *iter = next_keypoints[i];
      iter++;
    }
    cout << "tracked keypoints: " << keypoints.size() << endl;

    if (keypoints.size() == 0) {
      cout << "all keypoints are lost." << endl;
      break;
    }

    cv::Mat img_show = image.clone();
    for (auto kp : keypoints)
      cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
    cv::imshow("corners", img_show);
    cv::waitKey(1);
    last_image = image;
  }
}

void CamNode::StereoDepth(int *image_count, string *left_image_list,
                          string *right_image_list) {}
