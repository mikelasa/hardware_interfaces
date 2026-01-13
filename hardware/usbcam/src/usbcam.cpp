#include <usbcam/usbcam.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

struct Usbcam::Implementation {
  Usbcam::UsbcamConfig config{};
  RUT::TimePoint time0;

  cv::Mat image, image_cropped;
  std::shared_ptr<cv::VideoCapture> cap;

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0, const Usbcam::UsbcamConfig& config);
  cv::Mat next_rgb_frame_blocking();
};

Usbcam::Implementation::Implementation() {}
Usbcam::Implementation::~Implementation() {
  std::cout << "[Usbcam] finishing.." << std::endl;
}

bool Usbcam::Implementation::initialize(RUT::TimePoint time0,
                                       const Usbcam::UsbcamConfig& Usbcam_config) {
  std::cout << "[Usbcam] Initializing Usbcam pipeline.." << std::endl;
  time0 = time0;
  config = Usbcam_config;

  char* device_name_char = new char[config.device_name.length() + 1];
  strcpy(device_name_char, config.device_name.c_str());
  cap = std::make_shared<cv::VideoCapture>(device_name_char);
  if (!cap->isOpened()) {  //This section prompt an error message if no video stream is found//
    std::cerr << "\033[1;31mNo video stream detected. Check your device name "
                 "config\033[0m\n";
    std::cerr << "Current device name:" << config.device_name << std::endl;
    return false;
  }

  // config the video capture
  cap->set(cv::CAP_PROP_FRAME_WIDTH, config.frame_width);
  cap->set(cv::CAP_PROP_FRAME_HEIGHT, config.frame_height);
  cap->set(cv::CAP_PROP_FPS, config.fps);

  // try reading one frame
  std::cout << "Test reading a frame" << std::endl;
  *cap >> image;
  if (image.empty()) {
    std::cout << "\033[1;31mTest reading failed\033[0m\n";
    std::cout << "  Possibility one: Usbcam is not connected. " << std::endl;
    std::cout << "  Possibility two: Need to reset USB device. " << std::endl;
    std::cout << "    To do so, run 'lsusb | grep Elgato', which should give "
                 "something like\n";
    std::cout << "      Bus 010 Device 005: ID 0fd9:008a Elgato Systems GmbH "
                 "Elgato HD60 X\n";
    std::cout << "    Then run 'sudo ./usbreset /dev/bus/usb/XXX/YYY', where "
                 "XXX and YYY are the bus and device numbers above.\n";
    return false;
  }

  return true;
}