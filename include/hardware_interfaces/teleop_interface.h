/**
 * TeleopInterface: Base class for teleoperation devices
 * Provides common interface for different teleop hardware (SpaceMouse, joystick, etc.)
 */

#ifndef _TELEOP_INTERFACE_HEADER_
#define _TELEOP_INTERFACE_HEADER_

#include <RobotUtilities/spatial_utilities.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

struct TeleopData {
  // 6D input
  Eigen::Vector3d translation;  // X, Y, Z
  Eigen::Vector3d rotation;     // Roll, Pitch, Yaw
  
  // Button states
  std::vector<int> buttons;
  
  // Timestamp
  uint64_t timestamp;
  
  TeleopData() : translation(Eigen::Vector3d::Zero()), 
                 rotation(Eigen::Vector3d::Zero()),
                 timestamp(0) {}
};

class TeleopInterface {
 public:
  virtual ~TeleopInterface() {}

  /**
   * Initialize the teleoperation device
   * @return True if initialization successful
   */
  virtual bool init() = 0;

  /**
   * Cleanup and close device
   * @return True if cleanup successful
   */
  virtual bool cleanup() = 0;

  /**
   * Get the latest teleoperation input data
   * @param[out] data Structure containing 6D input
   * @return True if data is valid
   */
  virtual bool get_data(TeleopData& data) = 0;

  /**
   * Check if device is connected and operational
   * @return True if device is ready
   */
  virtual bool is_connected() const = 0;

};

#endif  // _TELEOP_INTERFACE_HEADER_
