#include "gello/gello.h"

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <chrono>
#include <cmath>
#include <iostream>

// Control table addresses (Protocol 2.0, XM/XC series)
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
static constexpr uint16_t LEN_PRESENT_VELOCITY  = 4;
static constexpr uint16_t LEN_PRESENT_POSITION  = 4;
// GroupSyncRead reads velocity + position in one burst
static constexpr uint16_t SYNC_READ_LEN = LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;

static inline dynamixel::PortHandler*   port(void* p)   { return static_cast<dynamixel::PortHandler*>(p); }
static inline dynamixel::PacketHandler* packet(void* p) { return static_cast<dynamixel::PacketHandler*>(p); }
static inline dynamixel::GroupSyncRead* gsr(void* p)    { return static_cast<dynamixel::GroupSyncRead*>(p); }

GelloInterface::GelloInterface() = default;

GelloInterface::~GelloInterface() { cleanup(); }

bool GelloInterface::init(const GelloConfig& cfg) {
  config_ = cfg;

  if (cfg.joint_ids.empty()) {
    std::cerr << "[Gello] joint_ids must not be empty\n";
    return false;
  }
  if (cfg.joint_ids.size() != cfg.joint_offsets.size() ||
      cfg.joint_ids.size() != cfg.joint_signs.size()) {
    std::cerr << "[Gello] joint_ids, joint_offsets, and joint_signs must have the same length\n";
    return false;
  }

  // Build the full ID list: arm joints + gripper
  all_ids_ = cfg.joint_ids;
  all_ids_.push_back(cfg.gripper_id);

  // Dynamixel SDK setup
  port_handler_   = dynamixel::PortHandler::getPortHandler(cfg.port.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0f);
  group_sync_read_ = new dynamixel::GroupSyncRead(
      port(port_handler_), packet(packet_handler_),
      ADDR_PRESENT_VELOCITY, SYNC_READ_LEN);

  if (!port(port_handler_)->openPort()) {
    std::cerr << "[Gello] Failed to open port " << cfg.port << "\n";
    return false;
  }
  if (!port(port_handler_)->setBaudRate(cfg.baudrate)) {
    std::cerr << "[Gello] Failed to set baudrate " << cfg.baudrate << "\n";
    port(port_handler_)->closePort();
    return false;
  }

  // Disable torque (GELLO is passive — it only sends, never receives commands)
  for (int id : all_ids_) {
    uint8_t dxl_error = 0;
    int result = packet(packet_handler_)->write1ByteTxRx(
        port(port_handler_), id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    if (result != COMM_SUCCESS) {
      std::cerr << "[Gello] Warning: could not disable torque for ID " << id << "\n";
    }
  }

  // Register all IDs with the sync reader
  for (int id : all_ids_) {
    if (!gsr(group_sync_read_)->addParam(id)) {
      std::cerr << "[Gello] Failed to add param for ID " << id << "\n";
      return false;
    }
  }

  // Initialise data container
  size_t n_joints = cfg.joint_ids.size();
  current_data_.joint_positions.assign(n_joints, 0.0);
  current_data_.gripper    = 0.0;
  current_data_.timestamp  = 0;   // reset so callers can detect when first real read arrives
  smoothed_positions_.assign(all_ids_.size(), 0.0);
  first_read_ = true;

  connected_ = true;
  running_   = true;
  read_thread_ = std::thread(&GelloInterface::read_loop, this);

  std::cout << "[Gello] Initialized on " << cfg.port
            << " with " << n_joints << " joints + gripper (ID " << cfg.gripper_id << ")\n";
  return true;
}

bool GelloInterface::cleanup() {
  if (!running_) return true;
  running_ = false;
  if (read_thread_.joinable()) read_thread_.join();

  if (group_sync_read_) { delete gsr(group_sync_read_); group_sync_read_ = nullptr; }
  if (port_handler_)    { port(port_handler_)->closePort(); port_handler_ = nullptr; }

  connected_ = false;
  return true;
}

bool GelloInterface::get_data(GelloData& data) {
  if (!connected_) return false;
  std::lock_guard<std::mutex> lock(data_mutex_);
  data = current_data_;
  return true;
}

bool GelloInterface::is_connected() const { return connected_; }

int GelloInterface::num_joints() const { return static_cast<int>(config_.joint_ids.size()); }

// ── private ──────────────────────────────────────────────────────────────────

double GelloInterface::raw_to_rad(int raw) const {
  // Dynamixel position 0–4095 maps to 0–2π; centre (2048) = π
  // raw * π / 2048
  return static_cast<double>(raw) * M_PI / 2048.0;
}

double GelloInterface::apply_gripper_mapping(double raw_rad) const {
  const double open  = config_.gripper_open_rad;
  const double close = config_.gripper_close_rad;
  if (std::abs(close - open) < 1e-9) return 0.0;
  double g = (raw_rad - open) / (close - open);
  return std::max(0.0, std::min(1.0, g));
}

void GelloInterface::read_loop() {
  const int sleep_us = (config_.read_rate_hz > 0)
                         ? (1000000 / config_.read_rate_hz)
                         : 2000;

  while (running_) {
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));

    int comm = gsr(group_sync_read_)->txRxPacket();
    if (comm != COMM_SUCCESS) {
      std::cerr << "[Gello] SyncRead comm error: " << comm << "\n";
      continue;
    }

    size_t n_arm     = config_.joint_ids.size();
    size_t n_total   = all_ids_.size();  // arm + gripper
    std::vector<double> raw_pos(n_total);

    bool ok = true;
    for (size_t i = 0; i < n_total; ++i) {
      int id = all_ids_[i];
      if (!gsr(group_sync_read_)->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
        ok = false;
        break;
      }
      int32_t raw = static_cast<int32_t>(
          gsr(group_sync_read_)->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
      // 32-bit two's complement correction
      if (raw > 0x7FFFFFFF) raw -= 0x100000000LL;
      raw_pos[i] = raw_to_rad(raw);
    }
    if (!ok) continue;

    // Exponential smoothing
    if (first_read_) {
      smoothed_positions_ = raw_pos;
      first_read_ = false;
    } else {
      double alpha = config_.smoothing_alpha;
      for (size_t i = 0; i < n_total; ++i)
        smoothed_positions_[i] = smoothed_positions_[i] * (1.0 - alpha) + raw_pos[i] * alpha;
    }

    // Apply offsets and signs to arm joints
    std::vector<double> joint_pos(n_arm);
    for (size_t i = 0; i < n_arm; ++i)
      joint_pos[i] = (smoothed_positions_[i] - config_.joint_offsets[i]) * config_.joint_signs[i];

    double gripper = apply_gripper_mapping(smoothed_positions_[n_arm]);

    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_data_.joint_positions = joint_pos;
      current_data_.gripper         = gripper;
      current_data_.timestamp       = static_cast<uint64_t>(now);
    }
  }
}