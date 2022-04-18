#include "image_subscriber/optical_flow.hpp"

#include <chrono>
#include <list>

void OpticalFlowTracker::SingleLKFlow(int *image_count, string *image_list) {
  list<Point2f> keypoints; // 因为要删除跟踪失败的点，使用list
  Mat image, last_image;

  for (int index = 100; index < *image_count; index++) {
    // 读入图像
    image = imread(*(image_list + index), IMREAD_COLOR);

    if (keypoints.size() < 100) {
      // 当特征点过少时重新提取特征点
      vector<KeyPoint> kps;
      Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();
      detector->detect(image, kps);

      //记录所有特征点
      for (auto kp : kps)
        keypoints.push_back(kp.pt);
      last_image = image;
      continue;
    }

    vector<Point2f> next_keypoints;
    vector<Point2f> prev_keypoints;
    for (auto kp : keypoints)
      prev_keypoints.push_back(kp);
    vector<unsigned char> status;

    vector<float> error;

    //调用OpenCV官方LK算法并计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    calcOpticalFlowPyrLK(last_image, image, prev_keypoints, next_keypoints,
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

    Mat img_show = image.clone();
    for (auto kp : keypoints)
      circle(img_show, kp, 10, Scalar(0, 240, 0), 1);
    imshow("corners", img_show);
    waitKey(1);
    last_image = image;
  }
}
