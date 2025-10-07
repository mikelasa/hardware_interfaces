/**
 * Realsense: wrapper around librealsense
 * https://github.com/IntelRealSense/librealsense
 *
 * Author:
 *      Yifan Hou <yifanhou@stanford.edu>
 */

#ifndef _REALSENSE_HEADER_
#define _REALSENSE_HEADER_

#include <yaml-cpp/yaml.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <RobotUtilities/timer_linux.h>

#include "hardware_interfaces/camera_interfaces.h"

class Realsense : public CameraInterfaces {
 public:
  struct RealsenseConfig {
    // Not all sizes and frame rates are supported by the camera.
    // Use realsense viewer to check what is supported.
    int width{1280};
    int height{720};
    int framerate{30};
    bool enable_color{true};
    bool enable_depth{true};
    bool align_depth_to_color{false};

    bool deserialize(const YAML::Node& node) {
      try {
        width = node["width"].as<int>();
        height = node["height"].as<int>();
        framerate = node["framerate"].as<int>();
        enable_color = node["enable_color"].as<bool>();
        enable_depth = node["enable_depth"].as<bool>();
        align_depth_to_color = node["align_depth_to_color"].as<bool>();
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  // for returning both rgb and depth frames from the same frameset
  struct FramePair {
      cv::Mat rgb;
      cv::Mat depth;
      bool valid;
  };

  Realsense();
  ~Realsense();

  /**
   * Initialize socket communication. Create a thread to run the 500Hz
   * communication with URe.
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(RUT::TimePoint time0, const RealsenseConfig& config);

  /**
   * Gets the next RGB image frame from the camera. This function blocks until
   * a new frame is available. frame is converted to cv::Mat before return.
   */
  cv::Mat next_rgb_frame_blocking() override;

  /**
   * Gets the next depth image frame from the camera. This function blocks until
   * a new frame is available. frame is converted to cv::Mat before return.
   */
  cv::Mat next_depth_frame_blocking();

  /**
   * Gets the next depth image and RGB frame from the camera. This function blocks until
   * a new frame is available. frame is converted to cv::Mat before return.
   */
  FramePair next_pair_frame_blocking();

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
  
};

#endif