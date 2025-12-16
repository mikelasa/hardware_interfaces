#include "gamepad/gamepad.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <cstdint>

Gamepad::Gamepad() : device_fd_(-1), running_(false), connected_(false) {}

Gamepad::~Gamepad() { cleanup(); }

bool Gamepad::init(const GamepadConfig& config) {
  config_ = config;
  std::cout << "[Gamepad] Initializing device at " << config_.device_path << "\n";

  // Open joystick device
  device_fd_ = open(config_.device_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (device_fd_ < 0) {
    std::cerr << "[Gamepad] Failed to open device " << config_.device_path << "\n";
    perror("open");
    return false;
  }

  // Get device name
  char name[128];
  if (ioctl(device_fd_, JSIOCGNAME(sizeof(name)), name) < 0) {
    std::cerr << "[Gamepad] Failed to get device name\n";
    close(device_fd_);
    device_fd_ = -1;
    return false;
  }

  device_info_ = std::string(name);
  std::cout << "[Gamepad] Device: " << device_info_ << "\n";

  // Get number of buttons and axes
  __u8 axes, buttons;
  ioctl(device_fd_, JSIOCGAXES, &axes);
  ioctl(device_fd_, JSIOCGBUTTONS, &buttons);
  std::cout << "[Gamepad] Axes: " << (int)axes << ", Buttons: " << (int)buttons << "\n";

  connected_ = true;
  running_ = true;
  read_thread_ = std::thread(&Gamepad::read_loop, this);

  std::cout << "[Gamepad] Device initialized\n";
  return true;
}

bool Gamepad::cleanup() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  if (device_fd_ >= 0) {
    close(device_fd_);
    device_fd_ = -1;
  }

  connected_ = false;
  return true;
}

bool Gamepad::get_data(GamepadData& data) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  data = current_data_;
  return connected_.load();
}

bool Gamepad::get_data(TeleopData& data) {
  GamepadData gp_data;
  if (!get_data(gp_data)) return false;

  // Map gamepad sticks to translation/rotation
  // Left stick = translation (X, Y)
  // Right stick = rotation (Roll, Pitch)
  data.translation << gp_data.left_stick_x, gp_data.left_stick_y, gp_data.left_trigger - gp_data.right_trigger;
  data.rotation << gp_data.right_stick_x, gp_data.right_stick_y, 0.0;
  
  // Store button states
  std::vector<int> buttons = {
    gp_data.button_a, gp_data.button_b, gp_data.button_x, gp_data.button_y,
    gp_data.button_lb, gp_data.button_rb, gp_data.button_back, gp_data.button_start
  };
  data.buttons = buttons;
  data.timestamp = gp_data.timestamp;
  return true;
}

bool Gamepad::is_connected() const { return connected_.load(); }

std::string Gamepad::get_device_info() const { return device_info_; }

void Gamepad::read_loop() {
  js_event event;
  const size_t event_size = sizeof(js_event);

  while (running_.load()) {
    // Read joystick event
    ssize_t bytes_read = read(device_fd_, &event, event_size);

    if (bytes_read == event_size) {
      // Process the event
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        parse_joystick_event(event);
        current_data_.timestamp = std::chrono::system_clock::now().time_since_epoch().count();
      }
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      // Error reading from device
      connected_ = false;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / config_.update_rate_hz));
  }
}

void Gamepad::parse_joystick_event(const js_event& event) {
  // Mask out JS_EVENT_INIT so both init and live events are handled the same way
  const uint8_t type = event.type & ~JS_EVENT_INIT;

  if (type == JS_EVENT_BUTTON) {
    // Button event
    int button_state = event.value ? 1 : 0;
    
    switch (event.number) {
      case 0: current_data_.button_a = button_state; break;
      case 1: current_data_.button_b = button_state; break;
      case 2: current_data_.button_x = button_state; break;
      case 3: current_data_.button_y = button_state; break;
      case 4: current_data_.button_lb = button_state; break;
      case 5: current_data_.button_rb = button_state; break;
      case 6: current_data_.button_back = button_state; break;
      case 7: current_data_.button_start = button_state; break;
      case 8: current_data_.button_left_stick = button_state; break;
      case 9: current_data_.button_right_stick = button_state; break;
      case 10: current_data_.dpad_up = button_state; break;
      case 11: current_data_.dpad_down = button_state; break;
      case 12: current_data_.dpad_left = button_state; break;
      case 13: current_data_.dpad_right = button_state; break;
    }
  } else if (type == JS_EVENT_AXIS) {
    // Axis event - normalize from int16 range to -1.0 to 1.0
    double normalized = event.value / 32768.0;
    
    switch (event.number) {
      case 0:  // Left stick X
        current_data_.left_stick_x = normalized;
        apply_deadzone(current_data_.left_stick_x, config_.deadzone_stick);
        break;
      case 1:  // Left stick Y
        current_data_.left_stick_y = -normalized;  // Invert for natural controls
        apply_deadzone(current_data_.left_stick_y, config_.deadzone_stick);
        break;
      case 2:  // Left trigger (optional, some controllers map to axis)
        current_data_.left_trigger = (normalized + 1.0) / 2.0;  // Convert -1..1 to 0..1
        apply_deadzone(current_data_.left_trigger, config_.deadzone_trigger);
        break;
      case 3:  // Right stick X
        current_data_.right_stick_x = normalized;
        apply_deadzone(current_data_.right_stick_x, config_.deadzone_stick);
        break;
      case 4:  // Right stick Y
        current_data_.right_stick_y = -normalized;  // Invert for natural controls
        apply_deadzone(current_data_.right_stick_y, config_.deadzone_stick);
        break;
      case 5:  // Right trigger (optional, some controllers map to axis)
        current_data_.right_trigger = (normalized + 1.0) / 2.0;  // Convert -1..1 to 0..1
        apply_deadzone(current_data_.right_trigger, config_.deadzone_trigger);
        break;
      case 6:  // D-pad horizontal as axis (common on some drivers)
        current_data_.dpad_left = (normalized < -0.5) ? 1 : 0;
        current_data_.dpad_right = (normalized > 0.5) ? 1 : 0;
        break;
      case 7:  // D-pad vertical as axis (common on some drivers)
        current_data_.dpad_up = (normalized < -0.5) ? 1 : 0;
        current_data_.dpad_down = (normalized > 0.5) ? 1 : 0;
        break;
    }
  }
}

void Gamepad::apply_deadzone(double& value, double threshold) {
  if (std::abs(value) < threshold) {
    value = 0.0;
  } else {
    // Scale the value to remove the deadzone gap
    if (value > 0) {
      value = (value - threshold) / (1.0 - threshold);
    } else {
      value = (value + threshold) / (1.0 - threshold);
    }
  }
}
