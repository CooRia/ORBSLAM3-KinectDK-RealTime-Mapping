/******************************************************************************
 * 文件: rgbd_azure_kinect_dk.cpp
 *
 * 本程序基于ORBSLAM3框架，使用Kinect相机进行实时的RGB-D建图。程序通过读取Kinect相机的
 * RGB图像和深度图像，利用ORBSLAM3算法进行实时的三维环境重建与定位。
 * 该程序参考了BAD-SLAM对于azure kinect相机的输入处理。
 *
 * 用法:
 * ./rgbd_azure_kinect_dk path_to_vocabulary path_to_settings (trajectory_file_name)
 *
 * 注意:
 * 1. 在CMakeLists中添加azure kinect的依赖(k4a)。
 * 2. 确保Kinect相机正确连接并驱动正常。
 * 3. 运行时请确保有足够的计算资源以保持实时性。
 *
 * - 2025.3.9
 *
 ******************************************************************************/

/******************************************************************************
 * File: rgbd_azure_kinect_dk.cpp
 *
 * Description:
 * This program is based on the ORBSLAM3 framework and uses a Kinect camera for
 * real-time RGB-D mapping. The program reads RGB and depth images from the Kinect
 * camera and utilizes the ORBSLAM3 algorithm for real-time 3D environment
 * reconstruction and localization.
 * The program references the input processing of the Azure Kinect camera from BAD-SLAM.
 *
 * Usage:
 * ./rgbd_azure_kinect_dk path_to_vocabulary path_to_settings (trajectory_file_name)
 *
 * Notes:
 * 1. Add the Azure Kinect dependency (k4a) in CMakeLists.txt.
 * 2. Ensure the Kinect camera is properly connected and drivers are installed.
 * 3. Ensure sufficient computational resources are available for real-time performance.
 *
 * - 2025.3.9
 ******************************************************************************/

#include <iostream>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <k4a/k4a.h>
// #include <k4arecord/playback.h>
// #include <k4arecord/record.h>

#include <System.h>

k4a_device_t device = nullptr;
k4a_capture_t capture;
k4a_calibration_t calibration;
k4a_transformation_t transformation;
k4a_image_t uncompressed_color_image;
k4a_device_configuration_t config{K4A_DEVICE_CONFIG_INIT_DISABLE_ALL};

int width;
int height;
float factor{1.0}; // scaling factor

cv::Mat camera_matrix;
cv::Mat new_camera_matrix;
cv::Mat cv_undistorted_color;
cv::Mat cv_undistorted_color_noalpha;
cv::Mat cv_color_downscaled;
cv::Mat map1;
cv::Mat map2;

void print_extrinsic(k4a_calibration_extrinsics_t *extrinsics)
{
  printf("R:\n");
  printf(" \
%.10f %.10f %.10f\n \
%.10f %.10f %.10f\n \
%.10f %.10f %.10f\n",
         extrinsics->rotation[0],
         extrinsics->rotation[1],
         extrinsics->rotation[2],
         extrinsics->rotation[3],
         extrinsics->rotation[4],
         extrinsics->rotation[5],
         extrinsics->rotation[6],
         extrinsics->rotation[7],
         extrinsics->rotation[8]);
  printf("t:\n");
  printf(
      "%.10f %.10f %.10f\n",
      extrinsics->translation[0],
      extrinsics->translation[1],
      extrinsics->translation[2]);
}

static void print_calibration(k4a_calibration_camera_t *calibration)
{
  printf("intrinsic parameters: \n");
  printf("resolution width: %d\n", calibration->resolution_width);
  printf("resolution height: %d\n", calibration->resolution_height);
  printf("principal point x: %.10f\n", calibration->intrinsics.parameters.param.cx);
  printf("principal point y: %.10f\n", calibration->intrinsics.parameters.param.cy);
  printf("focal length x: %.10f\n", calibration->intrinsics.parameters.param.fx);
  printf("focal length y: %.10f\n", calibration->intrinsics.parameters.param.fy);
  printf("K:\n");
  printf(" \
%.10f 0 %.10f\n \
0 %.10f %.10f\n \
0 0 1\n",
         calibration->intrinsics.parameters.param.fx,
         calibration->intrinsics.parameters.param.cx,
         calibration->intrinsics.parameters.param.fy,
         calibration->intrinsics.parameters.param.cy);
  printf("radial distortion coefficients:\n");
  printf("k1: %.10f\n", calibration->intrinsics.parameters.param.k1);
  printf("k2: %.10f\n", calibration->intrinsics.parameters.param.k2);
  printf("k3: %.10f\n", calibration->intrinsics.parameters.param.k3);
  printf("k4: %.10f\n", calibration->intrinsics.parameters.param.k4);
  printf("k5: %.10f\n", calibration->intrinsics.parameters.param.k5);
  printf("k6: %.10f\n", calibration->intrinsics.parameters.param.k6);
  printf("center of distortion in Z=1 plane, x: %.10f\n", calibration->intrinsics.parameters.param.codx);
  printf("center of distortion in Z=1 plane, y: %.10f\n", calibration->intrinsics.parameters.param.cody);
  printf("tangential distortion coefficient x: %.10f\n", calibration->intrinsics.parameters.param.p1);
  printf("tangential distortion coefficient y: %.10f\n", calibration->intrinsics.parameters.param.p2);
  printf("metric radius: %.10f\n", calibration->intrinsics.parameters.param.metric_radius);
  printf("extrinsic parameters: \n");
  print_extrinsic(&calibration->extrinsics);
  printf("\n");
}

bool undistort_rgb(
    k4a_calibration_intrinsic_parameters_t & /*intrinsics*/,
    const cv::Mat &cv_color,
    cv::Mat &undistorted_color)
{

  cv::resize(cv_color,
             cv_color_downscaled,
             cv_color_downscaled.size(),
             CV_INTER_AREA);
  // cv::imshow("cv color downscale ", cv_color_downscaled);
  remap(cv_color_downscaled, undistorted_color, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  return true;
}

void init_undistortion_map()
{
  auto intrinsics = calibration.color_camera_calibration.intrinsics.parameters.param;

  std::vector<double> _camera_matrix = {
      intrinsics.fx / factor,
      0.f,
      intrinsics.cx / factor,
      0.f,
      intrinsics.fy / factor,
      intrinsics.cy / factor,
      0.f,
      0.f,
      1.f};

  // Create cv matrices
  camera_matrix = cv::Mat(3, 3, CV_64F, &_camera_matrix[0]);
  // cv::Mat pinhole_camera_matrix = cv::Mat(3, 3, CV_32F, &_pinhole_camera_matrix[0]);

  std::vector<double> _dist_coeffs = {intrinsics.k1, intrinsics.k2, intrinsics.p1,
                                      intrinsics.p2, intrinsics.k3, intrinsics.k4,
                                      intrinsics.k5, intrinsics.k6};

  cv_color_downscaled = cv::Mat::zeros(
      height / factor,
      width / factor, CV_16U);

  cv::Mat dist_coeffs = cv::Mat(8, 1, CV_64F, &_dist_coeffs[0]);
  new_camera_matrix = cv::getOptimalNewCameraMatrix(
      camera_matrix,
      dist_coeffs,
      cv_color_downscaled.size(),
      0,
      cv_color_downscaled.size());
  std::cout << "Camera matrix is " << camera_matrix << std::endl;
  std::cout << "New camera matrix is " << new_camera_matrix << std::endl;
  std::cout << "Image resolution: " << width << " * " << height << std::endl;

  cv::Mat_<double> I = cv::Mat_<double>::eye(3, 3);

  map1 = cv::Mat::zeros(cv_color_downscaled.size(), CV_16SC2);
  map2 = cv::Mat::zeros(cv_color_downscaled.size(), CV_16UC1);
  initUndistortRectifyMap(camera_matrix, dist_coeffs, I, new_camera_matrix, cv::Size(width / factor, height / factor),
                          map1.type(), map1, map2);
  std::cout << "Map1 size is " << map1.size() << std::endl;
  ;
  std::cout << "map2 size is " << map1.size() << std::endl;
}

void init_memory()
{
  cv_undistorted_color = cv::Mat::zeros(
      height / factor,
      width / factor,
      CV_8UC4);

  cv_color_downscaled = cv::Mat::zeros(
      height / factor,
      width / factor,
      CV_8UC4);
}

k4a_fps_t k4a_convert_uint_to_fps(int fps)
{
  k4a_fps_t fps_enum;
  switch (fps)
  {
  case 5:
    fps_enum = K4A_FRAMES_PER_SECOND_5;
    break;
  case 15:
    fps_enum = K4A_FRAMES_PER_SECOND_15;
    break;
  case 30:
    fps_enum = K4A_FRAMES_PER_SECOND_30;
    break;
  default:
    fps_enum = K4A_FRAMES_PER_SECOND_30;
    break;
  }
  return fps_enum;
}

k4a_color_resolution_t k4a_convert_uint_to_resolution(int vertical_resolution)
{
  k4a_color_resolution_t resolution_enum;
  switch (vertical_resolution)
  {
  case 720:
    resolution_enum = K4A_COLOR_RESOLUTION_720P;
    break;
  case 1080:
    resolution_enum = K4A_COLOR_RESOLUTION_1080P;
    break;
  case 1440:
    resolution_enum = K4A_COLOR_RESOLUTION_1440P;
    break;
  case 1536:
    resolution_enum = K4A_COLOR_RESOLUTION_1536P;
    break;
  case 2160:
    resolution_enum = K4A_COLOR_RESOLUTION_2160P;
    break;
  case 3072:
    resolution_enum = K4A_COLOR_RESOLUTION_3072P;
    break;
  default:
    resolution_enum = K4A_COLOR_RESOLUTION_720P;
    break;
  }
  return resolution_enum;
}

int main(int argc, char **argv)
{
  if (argc < 3 || argc > 4)
  {
    std::cerr << std::endl
              << "Usage: ./mono_azure_kinect_dk path_to_vocabulary path_to_settings (trajectory_file_name)"
              << std::endl;
    return 1;
  }
  std::string file_name;
  file_name = std::string(argv[2]);

  // Check settings file
  cv::FileStorage fsSettings(file_name.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "Failed to open settings file at: " << file_name << std::endl;
    exit(-1);
  }
  std::cout << "Loading Kinect config file " << std::endl;

  int fps = 30;
  int resolution = 720;
  factor = 1.0;
  int exposure = 0; // exposure time

  cv::FileNode node = fsSettings["Kinect.resolution"];
  if (!node.empty() && node.isInt())
  {
    resolution = node.operator int();
  }

  node = fsSettings["Camera.fps"];
  if (!node.empty() && node.isInt())
  {
    fps = node.operator int();
  }

  node = fsSettings["Camera.imageScale"];
  if (!node.empty() && node.isReal())
  {
    factor = node.real();
  }

  uint32_t device_count = k4a_device_get_installed_count();
  if (device_count == 0)
  {
    std::cerr << "WARNING: Can't find any K4A device!" << std::endl;
    return 0;
  }

  if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
  {
    std::cerr << "WARNING: Failed to open K4A device!" << std::endl;
    return 0;
  }

  config.depth_mode = K4A_DEPTH_MODE_OFF;
  config.camera_fps = k4a_convert_uint_to_fps(fps);
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

  config.color_resolution = k4a_convert_uint_to_resolution(resolution);

  if (device)
  {
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
      std::cerr << "Failed to start K4A device cameras!" << std::endl;
      return 0;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
      std::cerr << "Failed to get calibration for K4A device!" << std::endl;
    }
  }

  transformation = k4a_transformation_create(&calibration);

  print_calibration(&calibration.color_camera_calibration);

  width = calibration.color_camera_calibration.resolution_width;
  height = calibration.color_camera_calibration.resolution_height;

  // Pre allocate memory
  init_memory();
  init_undistortion_map();

  int adjusted = 0;
  node = fsSettings["Adjusted"];
  if (!node.empty() && node.isInt())
  {
    adjusted = node.operator int();
  }

  fsSettings.release();
  std::cout << "Config file loaded successfully!" << std::endl;

  if (adjusted != 1)
  {
    std::cout << "Adjust Camera internal instrincts  in .yaml flle and then set 'Adjusted' to 1" << std::endl;

    std::cout << "Camera Matrix Format:\n";
    std::cout << "   fx   0   cx\n";
    std::cout << "   0   fy   cy\n";
    std::cout << "   0    0    1\n\n";
    std::cout << "Resolution:\n";
    std::cout << std::setw(10) << "720P  " << " : " << std::setw(8) << "1280 * 720   " << "  16:9\n";
    std::cout << std::setw(10) << "1080P" << " : " << std::setw(8) << "1920 * 1080" << "  16:9\n";
    std::cout << std::setw(10) << "1440P" << " : " << std::setw(8) << "2560 * 1440" << "  16:9\n";
    std::cout << std::setw(10) << "1536P" << " : " << std::setw(8) << "2048 * 1536" << "  4:3\n";
    std::cout << std::setw(10) << "2160P" << " : " << std::setw(8) << "3840 * 2160" << "  16:9\n";
    std::cout << std::setw(10) << "3072P" << " : " << std::setw(8) << "4096 * 3072" << "  4:3\n";
    return 0;
  }

  /*
    Camera Matrix
    fx  0  cx
    0  fy  cy
    0   0   1
  */

  /*
   Resolution
   720P : 1280 * 720  16:9
   1080P : 1920 * 1080 16:9
   1440P : 2560 * 1440 16:9
   1536P : 2048 * 1536 4:3
   2160P : 3840 * 2160 16:9
   3072P : 4096 * 3072 4:3
 */

  clock_t first_capture_start = clock();
  k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
  if (device)
  {
    // Wait for the first capture in a loop
    while (!(clock() - first_capture_start) < (CLOCKS_PER_SEC * 60))
    {
      result = k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE);
      if (result == K4A_WAIT_RESULT_SUCCEEDED)
      {
        k4a_capture_release(capture);
        break;
      }
      else if (result == K4A_WAIT_RESULT_FAILED)
      {
        std::cerr << "Runtime error: k4a_device_get_capture() returned error: " << result << std::endl;
      }
    }
  }
  std::cout << "successfully get the first capture" << std::endl;

  // ORB SLAM System
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true, 0, file_name);

  while (!SLAM.isShutDown())
  {

    result = k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE);
    if (result == K4A_WAIT_RESULT_FAILED)
    {
      std::cerr << "Failed to get capture from K4A device!" << std::endl;
      return 0;
    }

    // Access the rgb_image image
    k4a_image_t k4a_rgb_image = k4a_capture_get_color_image(capture);

    cv::Mat cv_uncompressed_color_image = cv::Mat(
        height,
        width,
        CV_8UC4,
        k4a_image_get_buffer(k4a_rgb_image));

    // Undistort (and downscale) color and depth
    undistort_rgb(
        calibration.color_camera_calibration.intrinsics.parameters,
        cv_uncompressed_color_image,
        cv_undistorted_color);

    double timestamp = k4a_image_get_device_timestamp_usec(k4a_rgb_image) / 1e6;
    cv::cvtColor(cv_undistorted_color, cv_undistorted_color, cv::COLOR_BGRA2BGR);
    SLAM.TrackMonocular(cv_undistorted_color, timestamp);

    if (k4a_rgb_image)
    {
      k4a_image_release(k4a_rgb_image);
    }
    k4a_capture_release(capture);
  }

  k4a_device_stop_cameras(device);
  if (device)
  {
    k4a_device_close(device);
    device = nullptr;
  }
}
