
#include "image_subscriber/cam_node.hpp"
#include "image_subscriber/optical_flow.hpp"

CamNode::CamNode() : Node("parameter_node") {
  SysInit();

  // Dynamic Image Count Array
  int image_count = GetImageCount(left_image_path_);
  string *left_image_list = new string[image_count];
  string *right_image_list = new string[image_count];
  GetImages(left_image_path_, image_count, left_image_list);
  GetImages(right_image_path_, image_count, right_image_list);

  // OpticalFlowTracker::SingleLKFlow(&image_count, left_image_list);
  StereoDepth(&image_count, left_image_list, right_image_list);
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
  namespace fs = std::filesystem;
  using fp = bool (*)(const std::filesystem::path &);
  return count_if(fs::directory_iterator(dir), fs::directory_iterator{},
                  (fp)fs::is_regular_file);
}

void CamNode::GetImages(string dir, int &image_count, string *image_list) {
  namespace fs = std::filesystem;
  int file_count = 0;
  for (auto const &dir_entry : fs::directory_iterator{dir}) {
    // cout << dir_entry << '\n';
    if (file_count < image_count)
      *(image_list + file_count++) = dir_entry.path();
  }
}

void CamNode::StereoDepth(int *image_count, string *left_image_list,
                          string *right_image_list) {
  cv::Mat left_image, right_image, depth_image;

  CAMERA_INTRINSIC_PARAMETERS left_camera = {
      458.654,
      457.296,
      367.215,
      248.375,
      -0.28340811,
      0.07395907,
      0.00019359,
      1.76187114e-05,
      {{0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975},
       {0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768},
       {-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949},
       {0.0, 0.0, 0.0, 1.0}}};
  CAMERA_INTRINSIC_PARAMETERS right_camera = {
      457.587,
      456.134,
      379.999,
      255.238,
      -0.28368365,
      0.07451284,
      -0.00010473,
      -3.55590700e-05,
      {{0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556},
       {0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024},
       {-0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038},
       {0.0, 0.0, 0.0, 1.0}},
  };

  Eigen::Matrix4d T_BC1;
  Eigen::Matrix4d T_BC2;
  T_BC1 << left_camera.T[0][0], left_camera.T[0][1], left_camera.T[0][2],
      left_camera.T[0][3], left_camera.T[1][0], left_camera.T[1][1],
      left_camera.T[1][2], left_camera.T[1][3], left_camera.T[2][0],
      left_camera.T[2][1], left_camera.T[2][2], left_camera.T[2][3],
      left_camera.T[3][0], left_camera.T[3][1], left_camera.T[3][2],
      left_camera.T[3][3];
  T_BC2 << right_camera.T[0][0], right_camera.T[0][1], right_camera.T[0][2],
      right_camera.T[0][3], right_camera.T[1][0], right_camera.T[1][1],
      right_camera.T[1][2], right_camera.T[1][3], right_camera.T[2][0],
      right_camera.T[2][1], right_camera.T[2][2], right_camera.T[2][3],
      right_camera.T[3][0], right_camera.T[3][1], right_camera.T[3][2],
      right_camera.T[3][3];
  cout.precision(15);

  Eigen::Matrix4d T_C1C2 = Eigen::Matrix4d::Zero();
  T_C1C2 = T_BC2.inverse() * T_BC1;

  for (int count = 2500; count < *image_count; count++) {
    left_image = cv::imread(*(left_image_list + count), cv::IMREAD_GRAYSCALE);
    right_image = cv::imread(*(right_image_list + count), cv::IMREAD_GRAYSCALE);

    // cv::imshow("undistort_left", left_image);
    // cv::imshow("undistort_right", right_image);
    undistort(left_image, right_image, left_camera, right_camera, T_C1C2);
    // cv::imshow("distort_left", left_image);
    // cv::imshow("distort_right", right_image);

    stereoSGBM(left_image, right_image, depth_image);
    // // disp2Depth(disp, depth_image, camera);
    // cv::imshow("image", left_image);
    // cv::imshow("depth", disp);

    // ??????c
    double fx = left_camera.fx;
    double fy = left_camera.fy;
    double cx = left_camera.cx;
    double cy = left_camera.cy;
    // ??????
    double b = 0.11;

    // ????????????
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    // ???????????????????????????????????????v++???u++??????v+=2, u+=2
    for (int v = 0; v < left_image.rows; v++)
      for (int u = 0; u < left_image.cols; u++) {
        if (depth_image.at<float>(v, u) <= 0.0 ||
            depth_image.at<float>(v, u) >= 96.0)
          continue;

        Vector4d point(0, 0, 0,
                       left_image.at<uchar>(v, u) /
                           255.0); // ????????????xyz,??????????????????

        // ???????????????????????? point ?????????
        double x = (u - cx) / fx;
        double y = (v - cy) / fy;
        double depth = fx * b / (depth_image.at<float>(v, u));
        point[0] = x * depth;
        point[1] = y * depth;
        point[2] = depth;

        pointcloud.push_back(point);
      }

    cv::imshow("pointcloud", depth_image / 96.0);
    cv::waitKey(0);
    // ????????????
    showPointCloud(pointcloud);
  }
}

void CamNode::undistort(cv::Mat &left_image, cv::Mat &right_image,
                        CAMERA_INTRINSIC_PARAMETERS left_camera,
                        CAMERA_INTRINSIC_PARAMETERS right_camera,
                        Eigen::Matrix4d T_BS) {
  cv::Mat left_cameraMatrix, left_distCoeffs, right_cameraMatrix,
      right_distCoeffs, R, T;
  left_cameraMatrix =
      (cv::Mat_<double>(3, 3) << left_camera.fx, 0, left_camera.cx, 0,
       left_camera.fy, left_camera.cy, 0, 0, 1);
  left_distCoeffs = (cv::Mat_<double>(5, 1) << left_camera.k1, left_camera.k2,
                     left_camera.p1, left_camera.p2, 0);
  right_cameraMatrix =
      (cv::Mat_<double>(3, 3) << right_camera.fx, 0, right_camera.cx, 0,
       right_camera.fy, right_camera.cy, 0, 0, 1);
  right_distCoeffs = (cv::Mat_<double>(5, 1) << right_camera.k1,
                      right_camera.k2, right_camera.p1, right_camera.p2, 0);
  // cv::Mat Rx;
  R = (cv::Mat_<double>(3, 3) << T_BS(0, 0), T_BS(0, 1), T_BS(0, 2), T_BS(1, 0),
       T_BS(1, 1), T_BS(1, 2), T_BS(2, 0), T_BS(2, 1), T_BS(2, 2));
  T = (cv::Mat_<double>(3, 1) << T_BS(0, 3), T_BS(1, 3), T_BS(2, 3));

  cv::Mat R1, R2, P1, P2, Q;
  // cv::Rect validRoi[2];
  cv::stereoRectify(left_cameraMatrix, left_distCoeffs, right_cameraMatrix,
                    right_distCoeffs, left_image.size(), R, T, R1, R2, P1, P2,
                    Q);
  cv::Mat map1x, map1y, map2x, map2y;

  initUndistortRectifyMap(left_cameraMatrix, left_distCoeffs, R1, P1,
                          left_image.size(), CV_32FC1, map1x, map1y);
  initUndistortRectifyMap(right_cameraMatrix, right_distCoeffs, R2, P2,
                          left_image.size(), CV_32FC1, map2x, map2y);
  remap(left_image, left_image, map1x, map1y, INTER_LINEAR); //?????????????????????
  remap(right_image, right_image, map2x, map2y, INTER_LINEAR);
}

void CamNode::stereoSGBM(cv::Mat left_image, cv::Mat right_image,
                         cv::Mat &disp) {
  disp.create(left_image.rows, left_image.cols, CV_8UC1);
  cv::Mat disp1 = cv::Mat(left_image.rows, left_image.cols, CV_16S);
  cv::Size imgSize = left_image.size();

  int nmDisparities = ((imgSize.width / 8) + 15) & -16; //??????????????????
  int pngChannels = left_image.channels(); //????????????????????????

  int mindisparity = 0;
  int ndisparities = 32;
  int SADWindowSize = 5;

  cv::Ptr<cv::StereoSGBM> sgbm =
      cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);

  int P1 = 8 * pngChannels * SADWindowSize * SADWindowSize;
  int P2 = 32 * pngChannels * SADWindowSize * SADWindowSize;

  sgbm->setPreFilterCap(15); //???????????????????????????
  // sgbm->setBlockSize(SADWindowSize); // SAD????????????
  sgbm->setP1(P1); //?????????????????????????????????
  sgbm->setP2(P2); //?????????????????????????????????                        //????????????
  sgbm->setNumDisparities(nmDisparities); //??????????????????
  sgbm->setUniquenessRatio(5);            //????????????????????????
  sgbm->setSpeckleWindowSize(15); //????????????????????????????????????????????????
  sgbm->setSpeckleRange(5);       //??????????????????
  sgbm->setDisp12MaxDiff(5);      //?????????????????????????????????
  sgbm->setMode(cv::StereoSGBM::MODE_SGBM); //??????????????????????????????????????????
  sgbm->compute(left_image, right_image, disp1);

  // disp1.convertTo(disp, CV_8U, 255 / (nmDisparities * 16.)); //???8???
  disp1.convertTo(disp, CV_32F, 255 / (nmDisparities * 16.));
}

// void CamNode::disp2Depth(cv::Mat disp, cv::Mat &depth,
//                          CAMERA_INTRINSIC_PARAMETERS camera) {
//   // // depth.create(disp.rows, disp.cols, CV_16S);
//   // cv::Mat depth1 = cv::Mat(disp.rows, disp.cols, CV_16S);
//   // for (int i = 0; i < disp.rows; i++) {
//   //   for (int j = 0; j < disp.cols; j++) {
//   //     if (!disp.ptr<ushort>(i)[j]) //?????????0??????
//   //       continue;
//   //     depth1.ptr<ushort>(i)[j] =
//   //         camera.scale * camera.fx * camera.baseline /
//   //         disp.ptr<ushort>(i)[j];
//   //   }
//   // }
//   // depth1.convertTo(depth, CV_8U, 1. / 255); //???8???
// }

void CamNode::showPointCloud(
    const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

  if (pointcloud.empty()) {
    cerr << "Point cloud is empty!" << endl;
    return;
  }

  pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto &p : pointcloud) {
      glColor3f(p[3], p[3], p[3]);
      glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();
    pangolin::FinishFrame();
    usleep(5000); // sleep 5 ms
  }
  return;
}
