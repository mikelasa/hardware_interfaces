#include "spacemouse/spacemouse.h"
#include <iostream>
#include <cmath>
#include <chrono>

SpaceMouse::SpaceMouse() : device_(nullptr), running_(false), connected_(false) {}

SpaceMouse::~SpaceMouse() { cleanup(); }

bool SpaceMouse::init(const SpaceMouseConfig& config) {
  config_ = config;
  std::cout << "[SpaceMouse] Initializing device...\n";

  if (hid_init() != 0) {
    std::cerr << "[SpaceMouse] Failed to init HID\n";
    return false;
  }

  struct hid_device_info* devs = hid_enumerate(config_.vendor_id, config_.product_id);
  if (!devs) {
    std::cerr << "[SpaceMouse] Device not found (0x" << std::hex << config_.vendor_id 
              << ":0x" << config_.product_id << std::dec << ")\n";
    hid_exit();
    return false;
  }

  struct hid_device_info* target_dev = devs;
  for (int i = 0; i < config_.device_index && target_dev; ++i) {
    target_dev = target_dev->next;
  }

  if (!target_dev) {
    std::cerr << "[SpaceMouse] Device at index " << config_.device_index << " not found\n";
    hid_free_enumeration(devs);
    hid_exit();
    return false;
  }

  device_ = hid_open_path(target_dev->path);
  hid_free_enumeration(devs);

  if (!device_) {
    std::cerr << "[SpaceMouse] Failed to open device\n";
    hid_exit();
    return false;
  }

  hid_set_nonblocking(device_, 1);
  device_info_ = "SpaceMouse (0x046d:0xc603)";

  current_data_.buttons.resize(2, 0);
  filtered_data_.buttons.resize(2, 0);

  connected_ = true;
  running_ = true;
  read_thread_ = std::thread(&SpaceMouse::read_loop, this);

  std::cout << "[SpaceMouse] Device initialized\n";
  return true;
}

bool SpaceMouse::cleanup() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  if (device_) {
    hid_close(device_);
    device_ = nullptr;
  }

  hid_exit();
  connected_ = false;
  return true;
}

bool SpaceMouse::get_data(SpaceMouseData& data) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  data = config_.enable_filtering ? filtered_data_ : current_data_;
  return connected_.load();
}

bool SpaceMouse::get_data(TeleopData& data) {
  SpaceMouseData sm_data;
  if (!get_data(sm_data)) return false;

  data.translation << sm_data.tx, sm_data.ty, sm_data.tz;
  data.rotation << sm_data.rx, sm_data.ry, sm_data.rz;
  data.buttons = sm_data.buttons;
  data.timestamp = sm_data.timestamp;
  return true;
}

bool SpaceMouse::is_connected() const { return connected_.load(); }

std::string SpaceMouse::get_device_info() const { return device_info_; }

void SpaceMouse::read_loop() {
  unsigned char data[256];

  while (running_.load()) {
    int res = hid_read(device_, data, sizeof(data));

    if (res > 0) {
      SpaceMouseData raw_data;
      raw_data.buttons.resize(2, 0);
      
      // Copy previous button states
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        raw_data.buttons = current_data_.buttons;
      }
      
      parse_hid_report(data, res, raw_data);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_data_ = raw_data;
        current_data_.timestamp = std::chrono::system_clock::now().time_since_epoch().count();

        if (config_.enable_filtering) {
          apply_filter(raw_data.tx, filtered_data_.tx, config_.filter_alpha);
          apply_filter(raw_data.ty, filtered_data_.ty, config_.filter_alpha);
          apply_filter(raw_data.tz, filtered_data_.tz, config_.filter_alpha);
          apply_filter(raw_data.rx, filtered_data_.rx, config_.filter_alpha);
          apply_filter(raw_data.ry, filtered_data_.ry, config_.filter_alpha);
          apply_filter(raw_data.rz, filtered_data_.rz, config_.filter_alpha);
          filtered_data_.buttons = raw_data.buttons;
          filtered_data_.timestamp = current_data_.timestamp;
        }
      }
    } else if (res < 0) {
      connected_ = false;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / config_.update_rate_hz));
  }
}

void SpaceMouse::parse_hid_report(const unsigned char* data, int size, SpaceMouseData& sm_data) {
  if (size < 1) return;

  if (size < 7 && data[0] != 3) return;

  // Helper function to convert 2-byte buffer to signed int16
  auto convert_buffer = [](unsigned char byte1, unsigned char byte2) -> int16_t {
    return (int16_t)((byte2 << 8) | byte1);
  };

  const double max_val = 350.0;

  if (data[0] == 1) {  // Translation report
    int16_t tx_raw = convert_buffer(data[3], data[4]);  // X
    int16_t ty_raw = convert_buffer(data[1], data[2]);  // Y
    int16_t tz_raw = convert_buffer(data[5], data[6]);  // Z (inverted)
    
    sm_data.tx = (tx_raw / max_val) * config_.translation_scale;
    sm_data.ty = -(ty_raw / max_val) * config_.translation_scale;  // Invert Y
    sm_data.tz = -(tz_raw / max_val) * config_.translation_scale;  // Invert Z
    
    apply_dead_zone(sm_data.tx, config_.dead_zone);
    apply_dead_zone(sm_data.ty, config_.dead_zone);
    apply_dead_zone(sm_data.tz, config_.dead_zone);
    
  } else if (data[0] == 2) {  // Rotation report
    int16_t rx_raw = convert_buffer(data[3], data[4]);  // Roll
    int16_t ry_raw = convert_buffer(data[1], data[2]);  // Pitch
    int16_t rz_raw = convert_buffer(data[5], data[6]);  // Yaw (inverted)
    
    sm_data.rx = (rx_raw / max_val) * config_.rotation_scale;
    sm_data.ry = (ry_raw / max_val) * config_.rotation_scale;
    sm_data.rz = -(rz_raw / max_val) * config_.rotation_scale;  // Invert Yaw
    
    apply_dead_zone(sm_data.rx, config_.dead_zone);
    apply_dead_zone(sm_data.ry, config_.dead_zone);
    apply_dead_zone(sm_data.rz, config_.dead_zone);
    
  } else if (data[0] == 3) {  // Button/keyboard report
    // Byte 1 contains button states (bits 0-1 for buttons, higher bits for other keys)
    if (size >= 2) {
      sm_data.buttons[0] = (data[1] & 0x01) ? 1 : 0;  // Button 1
      sm_data.buttons[1] = (data[1] & 0x02) ? 1 : 0;  // Button 2
    }
  }
}

void SpaceMouse::apply_dead_zone(double& value, double threshold) {
  if (std::abs(value) < threshold) {
    value = 0.0;
  } else {
    if (value > 0) {
      value = (value - threshold) / (1.0 - threshold);
    } else {
      value = (value + threshold) / (1.0 - threshold);
    }
  }
}

void SpaceMouse::apply_filter(double new_value, double& filtered_value, double alpha) {
  filtered_value = alpha * new_value + (1.0 - alpha) * filtered_value;
}
