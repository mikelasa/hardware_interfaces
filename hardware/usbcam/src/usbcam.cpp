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
  this->time0 = time0;
  config = Usbcam_config;

  cv::setNumThreads(config.cv_num_threads);

  char* device_name_char = new char[config.device_name.length() + 1];
  strcpy(device_name_char, config.device_name.c_str());
  cap = std::make_shared<cv::VideoCapture>(device_name_char);
  if (!cap->isOpened()) {
    std::cerr << "\033[1;31mNo video stream detected. Check your device name "
                 "config\033[0m\n";
    std::cerr << "Current device name:" << config.device_name << std::endl;
    return false;
  }

  // config the video capture
  cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
  cap->set(cv::CAP_PROP_FRAME_WIDTH, config.frame_width);
  cap->set(cv::CAP_PROP_FRAME_HEIGHT, config.frame_height);
  cap->set(cv::CAP_PROP_FPS, config.fps);
  cap->set(cv::CAP_PROP_BUFFERSIZE, 3);

  if (config.manual_exposure) {
    cap->set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // 1 = manual, 3 = auto
    cap->set(cv::CAP_PROP_EXPOSURE, config.exposure_absolute);
  }

  double actual_width = cap->get(cv::CAP_PROP_FRAME_WIDTH);
  double actual_height = cap->get(cv::CAP_PROP_FRAME_HEIGHT);
  double actual_fps = cap->get(cv::CAP_PROP_FPS);
  double actual_exp = cap->get(cv::CAP_PROP_EXPOSURE);
  std::cout << "[Usbcam] Requested: " << config.frame_width << "x"
            << config.frame_height << "@" << config.fps << "fps" << std::endl;
  std::cout << "[Usbcam] Actual:    " << actual_width << "x"
            << actual_height << "@" << actual_fps << "fps" << std::endl;
  std::cout << "[Usbcam] Exposure:  " << actual_exp/10.0 << " ms" << std::endl;

  // try reading one frame
  std::cout << "[Usbcam] Test reading a frame" << std::endl;
  *cap >> image;
  if (image.empty()) {
    std::cerr << "\033[1;31m[Usbcam] Test reading failed\033[0m\n";
    std::cerr << "  Possibility one: Usbcam is not connected." << std::endl;
    std::cerr << "  Possibility two: Need to reset USB device." << std::endl;
    std::cerr << "    Run 'lsusb | grep -i camera' to find the device, then\n";
    std::cerr << "    run 'sudo ./usbreset /dev/bus/usb/XXX/YYY'.\n";
    return false;
  }

  std::cout << "[Usbcam] Pipeline started.\n";
  return true;
}

cv::Mat Usbcam::Implementation::next_rgb_frame_blocking() {
  *cap >> image;
  if (image.empty()) {
    std::cerr << "[Usbcam] Empty frame. Terminate" << std::endl;
    return cv::Mat();
  }
  if (config.crop_rows[0] >= 0 && config.crop_cols[0] >= 0) {
    image_cropped = image(cv::Range(config.crop_rows[0], config.crop_rows[1]),
                          cv::Range(config.crop_cols[0], config.crop_cols[1]));
    return image_cropped;
  } else {
    return image;
  }
}

Usbcam::Usbcam() : impl_{std::make_unique<Implementation>()} {}

Usbcam::~Usbcam() {}

bool Usbcam::init(RUT::TimePoint time0, const UsbcamConfig& config) {
  return impl_->initialize(time0, config);
}

cv::Mat Usbcam::next_rgb_frame_blocking() {
  return impl_->next_rgb_frame_blocking();
}
