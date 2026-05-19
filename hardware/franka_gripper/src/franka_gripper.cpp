#include "franka_gripper/franka_gripper.h"

#include <franka/exception.h>
#include <franka/gripper.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

struct FrankaGripper::Implementation {
  FrankaGripper::FrankaGripperConfig _config{};

  std::unique_ptr<franka::Gripper> _gripper;

  // Cached state — updated by control loop via readOnce()
  franka::GripperState _gripper_state{};
  std::mutex _state_mtx;

  // Command target — written by setJoints / setJointsPosForce
  double _cmd_pos{0.0};
  double _cmd_force{0.0};
  std::mutex _cmd_mtx;

  // Thread lifecycle
  bool _thread_should_be_running{false};
  std::mutex _thread_running_mtx;
  std::thread _thread;

  // Safety tracking
  Eigen::VectorXd _joints_set_prev{Eigen::VectorXd::Zero(1)};
  Eigen::VectorXd _joints_set_truncated{Eigen::VectorXd::Zero(1)};

  Implementation();
  ~Implementation();

  bool initialize(RUT::TimePoint time0,
                  const FrankaGripper::FrankaGripperConfig& config);
  bool checkJointTarget(RUT::VectorXd& joints_set);
  void controlLoop();
};

FrankaGripper::Implementation::Implementation() {}

FrankaGripper::Implementation::~Implementation() {
  std::cout << "[FrankaGripper] Shutting down..." << std::endl;
  {
    std::lock_guard<std::mutex> lock(_thread_running_mtx);
    _thread_should_be_running = false;
  }
  // Interrupt any ongoing blocking move() / grasp() so the thread can exit
  try {
    if (_gripper) _gripper->stop();
  } catch (...) {}
  if (_thread.joinable()) _thread.join();
  std::cout << "[FrankaGripper] Shut down complete." << std::endl;
}

bool FrankaGripper::Implementation::initialize(
    RUT::TimePoint time0,
    const FrankaGripper::FrankaGripperConfig& config) {
  _config = config;

  std::cout << "[FrankaGripper] Connecting to " << config.robot_ip
            << std::endl;
  try {
    _gripper = std::make_unique<franka::Gripper>(config.robot_ip);
  } catch (const franka::Exception& e) {
    std::cerr << "[FrankaGripper] Connection failed: " << e.what() << std::endl;
    return false;
  }
  std::cout << "[FrankaGripper] Connected." << std::endl;

  try {
    _gripper_state = _gripper->readOnce();
  } catch (const franka::Exception& e) {
    std::cerr << "[FrankaGripper] Initial readOnce failed: " << e.what()
              << std::endl;
    return false;
  }

  // Initialise command to current width so the thread issues no command on first iter
  _cmd_pos = _gripper_state.width;
  _cmd_force = 0.0;
  _joints_set_prev[0] = _gripper_state.width;

  _thread = std::thread(&FrankaGripper::Implementation::controlLoop, this);

  // Wait until the thread signals it is running
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_thread_running_mtx);
      if (_thread_should_be_running) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "[FrankaGripper] Initialized. Width: " << _gripper_state.width
            << " m  max: " << _gripper_state.max_width << " m." << std::endl;
  return true;
}

bool FrankaGripper::Implementation::checkJointTarget(
    RUT::VectorXd& joints_set) {
  if (_config.js_interface_config.incre_safety_mode !=
      RobotSafetyMode::SAFETY_MODE_NONE) {
    bool incre_safe = incre_safety_check(joints_set, _joints_set_prev,
                                         _config.js_interface_config.max_incre);
    if (!incre_safe) {
      std::cerr << "[FrankaGripper] Incremental safety check failed.\n";
      if (_config.js_interface_config.incre_safety_mode ==
          RobotSafetyMode::SAFETY_MODE_STOP) {
        return false;
      } else if (_config.js_interface_config.incre_safety_mode ==
                 RobotSafetyMode::SAFETY_MODE_TRUNCATE) {
        joints_set =
            _joints_set_prev +
            (joints_set - _joints_set_prev)
                .cwiseMin(_config.js_interface_config.max_incre)
                .cwiseMax(-_config.js_interface_config.max_incre);
      }
    }
  }

  bool zone_safe = range_safety_check(
      joints_set, _config.js_interface_config.safe_zone, _joints_set_truncated);
  if (!zone_safe) {
    if (_config.js_interface_config.range_safety_mode ==
        RobotSafetyMode::SAFETY_MODE_STOP) {
      std::cerr << "[FrankaGripper] Range safety check failed. Target: "
                << joints_set.transpose() << std::endl;
      return false;
    } else if (_config.js_interface_config.range_safety_mode ==
               RobotSafetyMode::SAFETY_MODE_TRUNCATE) {
      joints_set = _joints_set_truncated;
    }
  }
  return true;
}

void FrankaGripper::Implementation::controlLoop() {
  {
    std::lock_guard<std::mutex> lock(_thread_running_mtx);
    _thread_should_be_running = true;
  }

  // Track last issued command to detect changes
  double last_cmd_pos = _cmd_pos;
  double last_cmd_force = 0.0;
  double cmd_pos, cmd_force;

  std::cout << "[FrankaGripper] Control loop started." << std::endl;

  while (true) {
    {
      std::lock_guard<std::mutex> lock(_thread_running_mtx);
      if (!_thread_should_be_running) break;
    }

    {
      std::lock_guard<std::mutex> lock(_cmd_mtx);
      cmd_pos = _cmd_pos;
      cmd_force = _cmd_force;
    }

    const bool pos_changed =
        std::abs(cmd_pos - last_cmd_pos) > _config.position_threshold;
    const bool force_changed =
        std::abs(cmd_force - last_cmd_force) > _config.force_threshold;

    if (pos_changed || force_changed) {
      last_cmd_pos = cmd_pos;
      last_cmd_force = cmd_force;
      try {
        if (cmd_force > _config.force_threshold) {
          // grasp() returns false if no object detected — not an error for us
          _gripper->grasp(cmd_pos, _config.grasp_speed, cmd_force,
                          _config.grasp_epsilon_inner,
                          _config.grasp_epsilon_outer);
        } else {
          _gripper->move(cmd_pos, _config.move_speed);
        }
      } catch (const franka::Exception& e) {
        std::cerr << "[FrankaGripper] Command failed: " << e.what()
                  << std::endl;
      }
    } else {
      // No new command — keep cached state fresh
      try {
        franka::GripperState state = _gripper->readOnce();
        {
          std::lock_guard<std::mutex> lock(_state_mtx);
          _gripper_state = state;
        }
      } catch (const franka::Exception& e) {
        std::cerr << "[FrankaGripper] readOnce failed: " << e.what()
                  << std::endl;
      }
    }
  }

  std::cout << "[FrankaGripper] Control loop finished." << std::endl;
}

// =============================================================================
// Public class
// =============================================================================

FrankaGripper::FrankaGripper() : m_impl{std::make_unique<Implementation>()} {}
FrankaGripper::~FrankaGripper() {}

bool FrankaGripper::init(RUT::TimePoint time0,
                         const FrankaGripperConfig& config) {
  return m_impl->initialize(time0, config);
}

bool FrankaGripper::getJoints(RUT::VectorXd& joints) {
  assert(joints.size() == 1);
  std::lock_guard<std::mutex> lock(m_impl->_state_mtx);
  joints[0] = m_impl->_gripper_state.width;
  return true;
}

bool FrankaGripper::setJoints(const RUT::VectorXd& joints) {
  assert(joints.size() == 1);
  Eigen::VectorXd joints_processed = joints;
  if (!m_impl->checkJointTarget(joints_processed)) return false;
  m_impl->_joints_set_prev = joints_processed;
  {
    std::lock_guard<std::mutex> lock(m_impl->_cmd_mtx);
    m_impl->_cmd_pos = joints_processed[0];
    m_impl->_cmd_force = 0.0;
  }
  return true;
}

bool FrankaGripper::setJointsPosForce(const RUT::VectorXd& joints,
                                       const RUT::VectorXd& forces) {
  assert(joints.size() == 1);
  assert(forces.size() == 1);
  Eigen::VectorXd joints_processed = joints;
  if (!m_impl->checkJointTarget(joints_processed)) return false;
  m_impl->_joints_set_prev = joints_processed;
  {
    std::lock_guard<std::mutex> lock(m_impl->_cmd_mtx);
    m_impl->_cmd_pos = joints_processed[0];
    m_impl->_cmd_force = forces[0];
  }
  return true;
}
