#include "spacemouse/spacemouse.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
  std::cout << "SpaceMouse Test Program\n\n";

  SpaceMouse spacemouse;
  SpaceMouse::SpaceMouseConfig config;
  
  config.vendor_id = 0x256f;
  config.product_id = 0xc635;
  config.update_rate_hz = 1000;
  
  std::cout << "Initializing SpaceMouse...\n";
  if (!spacemouse.init(config)) {
    std::cerr << "Failed to initialize SpaceMouse\n";
    std::cerr << "Check: lsusb | grep 046d\n";
    return 1;
  }

  std::cout << "Device: " << spacemouse.get_device_info() << "\n\n";
  std::cout << "Reading for 10 seconds (move device and press buttons):\n\n";

  SpaceMouseData prev_data;
  prev_data.buttons.resize(2, 0);
  
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    SpaceMouseData data;
    if (spacemouse.get_data(data)) {
      // Print movement
      if (std::abs(data.tx) > 0.01 || std::abs(data.ty) > 0.01 || 
          std::abs(data.tz) > 0.01 || std::abs(data.rx) > 0.01 ||
          std::abs(data.ry) > 0.01 || std::abs(data.rz) > 0.01) {
        printf("Motion: TX:%.2f TY:%.2f TZ:%.2f RX:%.2f RY:%.2f RZ:%.2f\n",
               data.tx, data.ty, data.tz, data.rx, data.ry, data.rz);
      }
      
      prev_data = data;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << "\nTest complete!\n";
  spacemouse.cleanup();
  return 0;
}
