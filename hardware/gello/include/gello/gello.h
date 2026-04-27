#ifndef _GELLO_HEADER_
#define _GELLO_HEADER_

#include <yaml-cpp/yaml.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <cstdint>

struct GelloData {
  std::vector<double> joint_positions;  // radians, offsets and signs applied
  double gripper;                       // 0.0 = open, 1.0 = closed
  uint64_t timestamp;

  GelloData() : gripper(0.0), timestamp(0) {}
};

class GelloInterface {
 public:
  struct GelloConfig {
    std::string port{"/dev/ttyACM1"};
    int baudrate{1000000};
    std::vector<int> joint_ids;           // Dynamixel servo IDs (base → wrist)
    std::vector<double> joint_offsets;    // per-joint offset in radians (multiples of π/2)
    std::vector<int> joint_signs;         // +1 or -1 per joint
    int gripper_id{8};
    double gripper_open_rad{0.0};         // position in rad when open
    double gripper_close_rad{0.0};        // position in rad when closed
    double smoothing_alpha{0.99};         // exponential smoothing (0=no smooth, 1=full smooth)
    int read_rate_hz{500};

    bool deserialize(const YAML::Node& node) {
      try {
        if (node["port"])           port = node["port"].as<std::string>();
        if (node["baudrate"])       baudrate = node["baudrate"].as<int>();
        if (node["joint_ids"])      joint_ids = node["joint_ids"].as<std::vector<int>>();
        if (node["joint_offsets"])  joint_offsets = node["joint_offsets"].as<std::vector<double>>();
        if (node["joint_signs"])    joint_signs = node["joint_signs"].as<std::vector<int>>();
        if (node["gripper_id"])     gripper_id = node["gripper_id"].as<int>();
        if (node["gripper_open_rad"])  gripper_open_rad = node["gripper_open_rad"].as<double>();
        if (node["gripper_close_rad"]) gripper_close_rad = node["gripper_close_rad"].as<double>();
        if (node["smoothing_alpha"]) smoothing_alpha = node["smoothing_alpha"].as<double>();
        if (node["read_rate_hz"])   read_rate_hz = node["read_rate_hz"].as<int>();
        return true;
      } catch (const std::exception& e) {
        return false;
      }
    }
  };

  GelloInterface();
  ~GelloInterface();

  bool init(const GelloConfig& config);
  bool cleanup();
  bool get_data(GelloData& data);
  bool is_connected() const;
  int num_joints() const;

 private:
  void read_loop();
  double raw_to_rad(int raw) const;
  double apply_gripper_mapping(double raw_rad) const;

  // Dynamixel SDK objects (forward-declared as void* to avoid header pollution)
  void* port_handler_{nullptr};
  void* packet_handler_{nullptr};
  void* group_sync_read_{nullptr};

  GelloConfig config_;
  GelloData current_data_;

  std::thread read_thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex data_mutex_;
  std::atomic<bool> connected_{false};

  // all IDs including gripper
  std::vector<int> all_ids_;
  std::vector<double> smoothed_positions_;
  bool first_read_{true};
};

#endif  // _GELLO_HEADER_