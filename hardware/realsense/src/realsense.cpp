#include "realsense/realsense.h"
#include "realsense/cv-helpers.hpp"

#include <iostream>
#include <librealsense2/rs.hpp>

struct Realsense::Implementation {
  Realsense::RealsenseConfig config{};
  RUT::TimePoint time0;

  rs2::pipeline pipe;  // Main RealSense pipeline
  std::shared_ptr<rs2::align> align_to_ptr; // Aligns depth to color coordinates
  rs2::frameset frames; // Container for multiple frames
  rs2::frame color_frame; // Individual color frame
  rs2::frame depth_frame; // Individual depth frame
  rs2::colorizer color_map; // color map for depth visualization

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0,
                  const Realsense::RealsenseConfig& config);
  cv::Mat next_rgb_frame_blocking();
  cv::Mat next_depth_frame_blocking();
  FramePair next_pair_frame_blocking();
};

Realsense::Implementation::Implementation() {}

Realsense::Implementation::~Implementation() {
  std::cout << "[Realsense] finishing.." << std::endl;
}

bool Realsense::Implementation::initialize(
    RUT::TimePoint time0, const Realsense::RealsenseConfig& realsense_config) {
  std::cout << "[Realsense] Initializing realsense pipeline.." << std::endl;
  time0 = time0;
  config = realsense_config;

  // creates an alignment objet to align the color and depth frames to color coordinate system.
  align_to_ptr = std::make_shared<rs2::align>(rs2_stream::RS2_STREAM_COLOR);

  // stream: https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93
  // format: https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#ae04b7887ce35d16dbd9d2d295d23aac7
  // also see this issue for acceptable formats: https://github.com/IntelRealSense/librealsense/issues/6341
  // rs2::config lets you configure the pipeline with the desired streams, the device is configured with the first stream that matches the requested configuration
  rs2::config rs_cfg;
  // if config has both color and depth enabled, use RS2_STREAM_ANY to let realsense decide the best format
  if (config.enable_color && !config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, config.width,
                         config.height, rs2_format::RS2_FORMAT_BGR8,
                         config.framerate);
  // if only depth is enabled, use Z16 format
  } else if (!config.enable_color && config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, config.width,
                         config.height, rs2_format::RS2_FORMAT_Z16,
                         config.framerate);
  // if both are enabled, use ANY for both
  } else if (config.enable_color && config.enable_depth) {
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, config.width,
                         config.height, rs2_format::RS2_FORMAT_BGR8,
                         config.framerate);
    rs_cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, config.width,
                         config.height, rs2_format::RS2_FORMAT_Z16,
                         config.framerate);
  } else {
    std::cerr << "[Realsense] Error: no stream enabled." << std::endl;
    return false;
  }
  pipe.start(rs_cfg);

  std::cout << "[Realsense] Pipeline started.\n";
  return true;
}

cv::Mat Realsense::Implementation::next_rgb_frame_blocking() {
  try {
    while (true) {
      // wait frames
      frames = pipe.wait_for_frames();
      // the alignment parameter is true, then align both sensors to color
      if (config.align_depth_to_color) {
        frames = align_to_ptr->process(frames);
      }
      //get the color frame
      color_frame = frames.get_color_frame();
      // If color frame did not update, continue
      // prevents duplicates when frame rate is too high
      static int last_frame_number = 0;
      if (color_frame.get_frame_number() == last_frame_number)
        continue;
      last_frame_number = static_cast<int>(color_frame.get_frame_number());
      break;
    }
    // Convert rs2::frame to cv::Mat
    return frame_to_mat(color_frame);

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return cv::Mat();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return cv::Mat();
  }
}

cv::Mat Realsense::Implementation::next_depth_frame_blocking() {
  try {
    while (true) {
      // wait frames
      frames = pipe.wait_for_frames();
      // the alignment parameter is true, then align both sensors to color
      if (config.align_depth_to_color) {
        frames = align_to_ptr->process(frames);
      }
      //get the depth frame
      depth_frame = frames.get_depth_frame();
      // If depth frame did not update, continue
      // prevents duplicates when frame rate is too high
      static int last_frame_number = 0;
      if (depth_frame.get_frame_number() == last_frame_number)
        continue;
      last_frame_number = static_cast<int>(depth_frame.get_frame_number());
      break;
    }
    // Convert rs2::frame to cv::Mat
    return depth_frame_to_meters(depth_frame);

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return cv::Mat();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return cv::Mat();
  }
}

Realsense::FramePair Realsense::Implementation::next_pair_frame_blocking() {
  FramePair pair;
  pair.valid = false;
  try {
    while (true) {
      // wait frames
      frames = pipe.wait_for_frames();
      // the alignment parameter is true, then align both sensors to color
      if (config.align_depth_to_color) {
        frames = align_to_ptr->process(frames);
      }
      //get the color and depth frames
      color_frame = frames.get_color_frame();
      depth_frame = frames.get_depth_frame();
      // If either frame did not update, continue
      // prevents duplicates when frame rate is too high
      static int last_color_frame_number = 0;
      static int last_depth_frame_number = 0;
      if (color_frame.get_frame_number() == last_color_frame_number ||
          depth_frame.get_frame_number() == last_depth_frame_number)
        continue;
      last_color_frame_number = static_cast<int>(color_frame.get_frame_number());
      last_depth_frame_number = static_cast<int>(depth_frame.get_frame_number());
      break;
    }
    // Convert rs2::frame to cv::Mat
    pair.rgb = frame_to_mat(color_frame);
    pair.depth = depth_frame_to_meters(depth_frame);
    pair.valid = true;
    return pair;

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return pair;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return pair;
  }
}

Realsense::Realsense() : m_impl{std::make_unique<Implementation>()} {}

Realsense::~Realsense() {}

bool Realsense::init(RUT::TimePoint time0,
                     const RealsenseConfig& realsense_config) {
  return m_impl->initialize(time0, realsense_config);
}

cv::Mat Realsense::next_rgb_frame_blocking() {
  return m_impl->next_rgb_frame_blocking();
}

cv::Mat Realsense::next_depth_frame_blocking() {
  return m_impl->next_depth_frame_blocking();
}

Realsense::FramePair Realsense::next_pair_frame_blocking() {
  return m_impl->next_pair_frame_blocking();
}