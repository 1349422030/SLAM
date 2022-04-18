#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/video/tracking.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

class OpticalFlowTracker {
public:
  friend class CamNode;
  OpticalFlowTracker();

  static void SingleLKFlow(int *image_count, string *image_list);

private:
};

#endif
