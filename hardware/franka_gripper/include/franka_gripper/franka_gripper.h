#ifndef _FRANKA_GRIPPER_HEADER_
#define _FRANKA_GRIPPER_HEADER_

#include <memory>
#include <string>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <yaml-cpp/yaml.h>

#include "hardware_interfaces/js_interfaces.h"

class FrankaGripper : public JSInterfaces {
 public:
  struct FrankaGripperConfig {
    std::string robot_ip{};
    double move_speed{0.1};            // m/s — speed for move()
    double grasp_speed{0.1};           // m/s — speed for grasp()
    double grasp_epsilon_inner{0.005}; // m — inner tolerance for grasp detection
    double grasp_epsilon_outer{0.005}; // m — outer tolerance for grasp detection
    double position_threshold{0.001};  // m — min width change to issue a new command
    double force_threshold{0.5};       // N — force above this triggers grasp() instead of move()
    JSInterfaceConfig js_interface_config{};

    bool deserialize(const YAML::Node& node) {
      try {
        robot_ip = node["robot_ip"].as<std::string>();
        move_speed = node["move_speed"].as<double>();
        grasp_speed = node["grasp_speed"].as<double>();
        grasp_epsilon_inner = node["grasp_epsilon_inner"].as<double>();
        grasp_epsilon_outer = node["grasp_epsilon_outer"].as<double>();
        position_threshold = node["position_threshold"].as<double>();
        force_threshold = node["force_threshold"].as<double>();
        js_interface_config.deserialize(node["js_interface_config"]);
      } catch (const std::exception& e) {
        std::cerr << "[FrankaGripper] Failed to load config: " << e.what()
                  << std::endl;
        return false;
      }
      return true;
    }
  };

  FrankaGripper();
  ~FrankaGripper();

  bool init(RUT::TimePoint time0, const FrankaGripperConfig& config);

  bool getJoints(RUT::VectorXd& joints) override;
  bool setJoints(const RUT::VectorXd& joints) override;
  bool setJointsPosForce(const RUT::VectorXd& joints,
                         const RUT::VectorXd& forces) override;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif  // _FRANKA_GRIPPER_HEADER_
