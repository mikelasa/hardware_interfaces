#include "spacemouse/spacemouse.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main() {
  std::cout << "SpaceMouse Debug - Raw HID Bytes\n\n";

  SpaceMouse spacemouse;
  SpaceMouse::SpaceMouseConfig config;
  
  config.vendor_id = 0x256f;
  config.product_id = 0xc635;
  config.update_rate_hz = 1000;
  
  std::cout << "Initializing SpaceMouse...\n";
  if (!spacemouse.init(config)) {
    std::cerr << "Failed to initialize SpaceMouse\n";
    return 1;
  }

  std::cout << "Device: " << spacemouse.get_device_info() << "\n";
  std::cout << "\n=== AXIS MAPPING TEST ===\n";
  std::cout << "TX (X-axis): Should change when you PUSH FORWARD (away from you)\n";
  std::cout << "TY (Y-axis): Should change when you PUSH LEFT/RIGHT (left to your left)\n";
  std::cout << "TZ (Z-axis): Should change when you PUSH UP (lift the device up)\n";
  std::cout << "\nStarting test now - do each motion slowly...\n\n";

  SpaceMouseData prev_data;
  prev_data.buttons.resize(2, 0);
  
  auto start = std::chrono::steady_clock::now();
  int report_count = 0;
  
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(20)) {
    SpaceMouseData data;
    if (spacemouse.get_data(data)) {
      // Only print when there's significant motion
      if (std::abs(data.tx) > 0.05 || std::abs(data.ty) > 0.05 || std::abs(data.tz) > 0.05 ||
          std::abs(data.rx) > 0.05 || std::abs(data.ry) > 0.05 || std::abs(data.rz) > 0.05) {
        report_count++;
        if (report_count <= 50) {  // Limit output
          std::cout << "Report " << std::setw(3) << report_count << ": ";
          std::cout << "TX=" << std::setw(7) << std::fixed << std::setprecision(3) << data.tx << " ";
          std::cout << "TY=" << std::setw(7) << std::fixed << std::setprecision(3) << data.ty << " ";
          std::cout << "TZ=" << std::setw(7) << std::fixed << std::setprecision(3) << data.tz << "  ";
          std::cout << "RX=" << std::setw(7) << std::fixed << std::setprecision(3) << data.rx << " ";
          std::cout << "RY=" << std::setw(7) << std::fixed << std::setprecision(3) << data.ry << " ";
          std::cout << "RZ=" << std::setw(7) << std::fixed << std::setprecision(3) << data.rz << "\n";
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << "\nDebug complete!\n";
  spacemouse.cleanup();
  return 0;
}
