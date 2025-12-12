/**
 * SpaceMouse: Teleoperation device wrapper for 3D SpaceMouse using HID API
 */

#ifndef _SPACEMOUSE_HEADER_
#define _SPACEMOUSE_HEADER_

#include <hardware_interfaces/teleop_interface.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <hidapi/hidapi.h>

struct SpaceMouseData {
  double tx, ty, tz;          // Translation
  double rx, ry, rz;          // Rotation
  std::vector<int> buttons;
  uint64_t timestamp;
  
  SpaceMouseData() : tx(0), ty(0), tz(0), rx(0), ry(0), rz(0), timestamp(0) {}
};

class SpaceMouse : public TeleopInterface {
 public:
  struct SpaceMouseConfig {
    unsigned short vendor_id{0x256f};
    unsigned short product_id{0xc635};
    int device_index{0};
    double translation_scale{1.0};
    double rotation_scale{1.0};
    double dead_zone{0.05};
    bool enable_filtering{true};
    double filter_alpha{0.3};
    int update_rate_hz{1000};

    bool deserialize(const YAML::Node& node) {
      try {
        if (node["vendor_id"]) vendor_id = node["vendor_id"].as<unsigned short>();
        if (node["product_id"]) product_id = node["product_id"].as<unsigned short>();
        if (node["device_index"]) device_index = node["device_index"].as<int>();
        if (node["translation_scale"]) translation_scale = node["translation_scale"].as<double>();
        if (node["rotation_scale"]) rotation_scale = node["rotation_scale"].as<double>();
        if (node["dead_zone"]) dead_zone = node["dead_zone"].as<double>();
        if (node["enable_filtering"]) enable_filtering = node["enable_filtering"].as<bool>();
        if (node["filter_alpha"]) filter_alpha = node["filter_alpha"].as<double>();
        if (node["update_rate_hz"]) update_rate_hz = node["update_rate_hz"].as<int>();
        return true;
      } catch (const std::exception& e) {
        std::cerr << "SpaceMouse config error: " << e.what() << std::endl;
        return false;
      }
    }
  };

  SpaceMouse();
  ~SpaceMouse();

  bool init(const SpaceMouseConfig& config);
  
  // TeleopInterface implementation
  bool init() override { return false; }  // Use init(config) instead
  bool cleanup() override;
  bool get_data(TeleopData& data) override;
  bool is_connected() const override;
  std::string get_device_info() const override;
  
  // SpaceMouse-specific methods
  bool get_data(SpaceMouseData& data);

 private:
  void read_loop();
  void parse_hid_report(const unsigned char* data, int size, SpaceMouseData& sm_data);
  void apply_dead_zone(double& value, double threshold);
  void apply_filter(double new_value, double& filtered_value, double alpha);

  hid_device* device_{nullptr};
  SpaceMouseData current_data_;
  SpaceMouseData filtered_data_;
  SpaceMouseConfig config_;
  
  std::thread read_thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex data_mutex_;
  std::atomic<bool> connected_{false};
  std::string device_info_;
};

#endif  // _SPACEMOUSE_HEADER_
