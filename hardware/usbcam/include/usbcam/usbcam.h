/**
 * Usbcam: video capture interface for a generic usb camera with a video capture card.
 *
 * Author:
 *      Mikel Lasa
 */

#ifndef _USBCAM_HEADER_
#define _USBCAM_HEADER_

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <RobotUtilities/timer_linux.h>

#include "hardware_interfaces/camera_interfaces.h"

class Usbcam : public CameraInterfaces {
 public:
  struct UsbcamConfig {
    std::string device_name;
    double frame_width{1280};
    double frame_height{720};
    std::vector<int> crop_rows{-1, -1};
    std::vector<int> crop_cols{-1, -1};
    int fps{30};
    bool manual_exposure{false};
    int exposure_absolute{200};  // 100µs units: 200 = 20ms = 1/50s
    int cv_num_threads{1};

    bool deserialize(const YAML::Node& node) {
      try {
        device_name = node["device_name"].as<std::string>();
        frame_width = node["frame_width"].as<double>();
        frame_height = node["frame_height"].as<double>();
        crop_rows = node["crop_rows"].as<std::vector<int>>();
        crop_cols = node["crop_cols"].as<std::vector<int>>();
        fps = node["fps"].as<int>();
        if (node["manual_exposure"])
          manual_exposure = node["manual_exposure"].as<bool>();
        if (node["exposure_absolute"])
          exposure_absolute = node["exposure_absolute"].as<int>();
        if (node["cv_num_threads"])
          cv_num_threads = node["cv_num_threads"].as<int>();
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  Usbcam();
  ~Usbcam();

  /**
   * Initialize socket communication. Create a thread to run the 500Hz
   * communication with URe.
   *
   * @param[in]  time0    Start time. Time will count from this number.
   * @param[in]  config   controller configs.
   *
   * @return     True if success.
   */
  bool init(RUT::TimePoint time0, const UsbcamConfig& config);
  cv::Mat next_rgb_frame_blocking() override;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> impl_;
};

#endif  // _USBCAM_HEADER_