#include "gello/gello.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

int main() {
  GelloInterface gello;

  GelloInterface::GelloConfig config;
  config.port      = "/dev/ttyACM0";
  config.baudrate  = 1000000;
  config.joint_ids = {1, 2, 3, 4, 5, 6, 7};
  // Calibrated offsets for Franka Panda (gello_get_offset.py, start_joints 0 0 0 -1.57 0 1.57 0)
  config.joint_offsets = {
   -2.0 * M_PI / 2.0,   // joint 1: -2*pi/2
    2.0 * M_PI / 2.0,   // joint 2:  2*pi/2
    2.0 * M_PI / 2.0,   // joint 3:  2*pi/2
    3.0 * M_PI / 2.0,   // joint 4:  3*pi/2
   -2.0 * M_PI / 2.0,   // joint 5: -2*pi/2
    3.0 * M_PI / 2.0,   // joint 6:  3*pi/2
    3.0 * M_PI / 2.0,   // joint 7:  3*pi/2
  };
  config.joint_signs       = {1, 1, 1, 1, 1, -1, 1};
  config.gripper_id        = 8;
  config.gripper_open_rad  = 103.598828125 * M_PI / 180.0;
  config.gripper_close_rad = 61.798828125  * M_PI / 180.0;

  if (!gello.init(config)) {
    std::cerr << "Failed to initialize GELLO\n";
    return 1;
  }

  std::cout << "Reading GELLO joints (Ctrl+C to stop)...\n";
  for (int i = 0; i < 200; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    GelloData data;
    if (gello.get_data(data)) {
      std::cout << "joints: [";
      for (size_t j = 0; j < data.joint_positions.size(); ++j) {
        std::cout << data.joint_positions[j];
        if (j + 1 < data.joint_positions.size()) std::cout << ", ";
      }
      std::cout << "]  gripper: " << data.gripper << "\n";
    }
  }

  gello.cleanup();
  return 0;
}