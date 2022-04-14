#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include <iostream>

using namespace std;

class OpticalFlowTracker {
public:
  friend class CamNode;
  OpticalFlowTracker();

  static void SingleLKFlow(int *image_count, string *image_list);
  // void StereoDepth(int *image_count, string *left_image_list,
  //                  string *right_image_list);

private:
};

#endif
