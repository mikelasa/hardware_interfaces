#include "gamepad/gamepad.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
  std::cout << "Xbox Gamepad Test Program\n\n";

  Gamepad gamepad;
  Gamepad::GamepadConfig config;

  // Xbox 360 controller device path
  config.device_path = "/dev/input/js0";
  config.update_rate_hz = 300;

  std::cout << "Initializing Xbox controller...\n";
  if (!gamepad.init(config)) {
    std::cerr << "Failed to initialize gamepad\n";
    return 1;
  }

  std::cout << "Device: " << gamepad.get_device_info() << "\n\n";
  std::cout << "Reading for 10 seconds (move sticks and press buttons):\n\n";

  GamepadData prev_data;

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    GamepadData data;
    if (gamepad.get_data(data)) {
      // Print stick movement
      if (std::abs(data.left_stick_x) > 0.1 || std::abs(data.left_stick_y) > 0.1 ||
          std::abs(data.right_stick_x) > 0.1 || std::abs(data.right_stick_y) > 0.1) {
        printf("Sticks: LX:%.2f LY:%.2f RX:%.2f RY:%.2f\n",
               data.left_stick_x, data.left_stick_y, data.right_stick_x, data.right_stick_y);
      }

      // Print trigger movement
      if (data.left_trigger > 0.1 || data.right_trigger > 0.1) {
        printf("Triggers: LT:%.2f RT:%.2f\n", data.left_trigger, data.right_trigger);
      }

      // Detect button presses
      if (data.button_a != prev_data.button_a && data.button_a) printf("  Button A pressed\n");
      if (data.button_b != prev_data.button_b && data.button_b) printf("  Button B pressed\n");
      if (data.button_x != prev_data.button_x && data.button_x) printf("  Button X pressed\n");
      if (data.button_y != prev_data.button_y && data.button_y) printf("  Button Y pressed\n");
      if (data.button_lb != prev_data.button_lb && data.button_lb) printf("  Button LB pressed\n");
      if (data.button_rb != prev_data.button_rb && data.button_rb) printf("  Button RB pressed\n");
      if (data.button_start != prev_data.button_start && data.button_start) printf("  Button Start pressed\n");
      if (data.button_back != prev_data.button_back && data.button_back) printf("  Button Back pressed\n");

      // Detect D-pad
      if (data.dpad_up != prev_data.dpad_up && data.dpad_up) printf("  D-Pad UP pressed\n");
      if (data.dpad_down != prev_data.dpad_down && data.dpad_down) printf("  D-Pad DOWN pressed\n");
      if (data.dpad_left != prev_data.dpad_left && data.dpad_left) printf("  D-Pad LEFT pressed\n");
      if (data.dpad_right != prev_data.dpad_right && data.dpad_right) printf("  D-Pad RIGHT pressed\n");

      if (data.button_a) {
        gamepad.set_rumble(0, 65535, 300); // weak motor on A
      }

      if (data.button_b) {
        gamepad.set_rumble(65535, 0, 300); // strong motor on B
      }
      prev_data = data;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "\nTest complete!\n";
  gamepad.cleanup();
  return 0;
}
