#include <franka/franka.h>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include "lowpass_filter.h"
#include "rate_limiting.h"
#include "robot_impl.h"
#include "network.h"

/**
 * Helper function to check if all values in an array are finite
 * @param array Array to check for finite values
 * @throws std::invalid_argument if any value is infinite or NaN
 */
template <typename T, size_t N>
inline void checkFinite(const std::array<T, N>& array) {
    if (!std::all_of(array.begin(), array.end(), [](double d) { return std::isfinite(d); })) {
        throw std::invalid_argument("Commanding value is infinite or NaN.");
    }
}

/**
 * Private implementation structure for FRANKA robot control
 * Contains all internal state and robot interface components
 */
struct FRANKA::Implementation {
    // Core robot components
    std::unique_ptr<franka::Robot::Impl> robot_impl;        // Pointer to robot implementation
    uint32_t motion_id{0};                                  // Current motion session ID
    franka::RobotState robot_state;                         // Last robot state
    FRANKA::FRANKAConfig config{};                         // Robot configuration from YAML

    // Command objects for robot communication
    research_interface::robot::MotionGeneratorCommand motion_command{};
    research_interface::robot::ControllerCommand control_command{};

    // Constructor and destructor
    Implementation();
    ~Implementation();

    // ========== Initialization ==========
    bool initialize(RUT::TimePoint time0, const FRANKA::FRANKAConfig& franka_config);

    // ========== State Getters (with readOnce) ==========
    bool getCartesian(RUT::Vector7d& pose_xyzq);
    bool getJoints(RUT::VectorXd& joints);
    bool getTorques(RUT::VectorXd& torques);
    bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
    bool getWrenchTool(RUT::Vector6d& wrench);
    bool getCartesianVelocity(RUT::Vector6d& velocity);

    // ========== State Setters ==========
    bool setCartesian(const RUT::Vector7d& pose_xyzq);
    bool setJoints(const RUT::VectorXd& joints);
    bool setTorques(const RUT::VectorXd& torques);

    // ========== Helper Methods (without readOnce) ==========
    bool getCurrentPose(RUT::Vector7d& pose_xyzq);
    bool getCurrentWrenchTool(RUT::Vector6d& wrench);
    franka::Duration getElapsedTime();
    franka::RobotState getRobotState();

    // ========== Low-Level Robot Interface ==========
    franka::RobotState readOnce();
    franka::RobotState update(const research_interface::robot::MotionGeneratorCommand* motion_command,
                              const research_interface::robot::ControllerCommand* control_command);
    franka::RealtimeConfig realtimeConfig() const noexcept;

    // ========== Motion Control ==========
    uint32_t startMotion(research_interface::robot::Move::ControllerMode controller_mode,
                         research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
                         const research_interface::robot::Move::Deviation& maximum_path_deviation,
                         const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation);
    void finishMotion(uint32_t motion_id,
                      const research_interface::robot::MotionGeneratorCommand* motion_command,
                      const research_interface::robot::ControllerCommand* control_command);
    void cancelMotion(uint32_t motion_id);
    void throwOnMotionError(const franka::RobotState& robot_state, uint32_t motion_id);
    void finishCurrentMotion();

    // ========== Robot Configuration ==========
    void setJointImpedance(const std::array<double, 7>& K_theta);
    void setCartesianImpedance(const std::array<double, 6>& K_x);
    void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                            const std::array<double, 7>& upper_torque_thresholds_acceleration,
                            const std::array<double, 7>& lower_torque_thresholds_nominal,
                            const std::array<double, 7>& upper_torque_thresholds_nominal,
                            const std::array<double, 6>& lower_force_thresholds_acceleration,
                            const std::array<double, 6>& upper_force_thresholds_acceleration,
                            const std::array<double, 6>& lower_force_thresholds_nominal,
                            const std::array<double, 6>& upper_force_thresholds_nominal);
    void setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,
               const std::array<double, 9>& load_inertia);

    // ========== Robot Model ==========
    franka::Model loadModel();
};

// ========== Implementation Constructor/Destructor ==========

FRANKA::Implementation::Implementation() {
    // Constructor implementation (if needed)
}

FRANKA::Implementation::~Implementation() {
    // Destructor implementation (if needed)
}

// ========== Implementation Initialization ==========

bool FRANKA::Implementation::initialize(RUT::TimePoint time0, const FRANKA::FRANKAConfig& config) {
    std::cout << "Robot IP: " << config.robot_ip << std::endl;

    // Create network connection to robot
    std::unique_ptr<franka::Network> network;
    try {
        std::cout << "Attempting to create Network connection..." << std::endl;
        network = std::make_unique<franka::Network>(
            config.robot_ip,
            research_interface::robot::kCommandPort);
        std::cout << "Network connection established successfully!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception while creating Network: " << e.what() << std::endl;
        return false;
    }
    
    // Configure realtime settings
    franka::RealtimeConfig rt_config = franka::RealtimeConfig::kIgnore;
    if (config.realtime_config == "ignore") {
        rt_config = franka::RealtimeConfig::kIgnore;
    } else if (config.realtime_config == "enforce") {
        rt_config = franka::RealtimeConfig::kEnforce;
    } 

    // Configure controller mode
    research_interface::robot::Move::ControllerMode cm_config = research_interface::robot::Move::ControllerMode::kExternalController;
    if (config.controller_mode == "joint_impedance") {
        cm_config = research_interface::robot::Move::ControllerMode::kJointImpedance;
    } else if (config.controller_mode == "cartesian_impedance") {
        cm_config = research_interface::robot::Move::ControllerMode::kCartesianImpedance;
    } else if (config.controller_mode == "external_controller") {
        cm_config = research_interface::robot::Move::ControllerMode::kExternalController;
    }

    // Configure motion generator mode
    research_interface::robot::Move::MotionGeneratorMode mg_config = research_interface::robot::Move::MotionGeneratorMode::kJointPosition;
    if (config.motion_generator_mode == "joint_position") {
        mg_config = research_interface::robot::Move::MotionGeneratorMode::kJointPosition;
    } else if (config.motion_generator_mode == "joint_velocity") {
        mg_config = research_interface::robot::Move::MotionGeneratorMode::kJointVelocity;
    } else if (config.motion_generator_mode == "cartesian_position") {
        mg_config = research_interface::robot::Move::MotionGeneratorMode::kCartesianPosition;
    } else if (config.motion_generator_mode == "cartesian_velocity") {
        mg_config = research_interface::robot::Move::MotionGeneratorMode::kCartesianVelocity;
    }

    // Create Robot implementation using network
    robot_impl = std::make_unique<franka::Robot::Impl>(
        std::move(network),
        config.log_size,
        rt_config);
    this->config = config;

    try {
        // Configure robot parameters
        setJointImpedance(this->config.setJointImpedance);
        setCartesianImpedance(this->config.setCartesianImpedance);
        setLoad(this->config.tcp_mass, this->config.fx_c_load, this->config.tcp_inertia);

        // Set collision detection thresholds
        const std::array<double, 7> lower_torque_thresholds_acceleration{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}};
        const std::array<double, 7> upper_torque_thresholds_acceleration{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}};
        const std::array<double, 7> lower_torque_thresholds_nominal{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}};
        const std::array<double, 7> upper_torque_thresholds_nominal{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}};
        const std::array<double, 6> lower_force_thresholds_acceleration{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}};
        const std::array<double, 6> upper_force_thresholds_acceleration{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}};
        const std::array<double, 6> lower_force_thresholds_nominal{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}};
        const std::array<double, 6> upper_force_thresholds_nominal{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}};

        setCollisionBehavior(lower_torque_thresholds_acceleration,
                             upper_torque_thresholds_acceleration,
                             lower_torque_thresholds_nominal,
                             upper_torque_thresholds_nominal,
                             lower_force_thresholds_acceleration,
                             upper_force_thresholds_acceleration,
                             lower_force_thresholds_nominal,
                             upper_force_thresholds_nominal);

        // Read initial robot state
        robot_state = readOnce();

        // Load robot model and compute initial jacobian
        franka::Model model(loadModel());
        const std::array<double, 42> initial_jacobian =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        (void)initial_jacobian; // Suppress unused variable warning

        // Set path deviation limits
        const research_interface::robot::Move::Deviation deviation_limit{
            this->config.deviation[0],
            this->config.deviation[1],
            this->config.deviation[2]};

        // Start initial motion session
        const uint32_t motion_id = startMotion(cm_config, mg_config, deviation_limit, deviation_limit);
        if (motion_id == 0) {
            std::cerr << "[ERROR] Failed to start Franka motion session." << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to configure Franka robot: " << e.what() << std::endl;
        return false;
    }

    return true;
}

// ========== FRANKA Class Implementation ==========

FRANKA::FRANKA() : impl_(std::make_unique<Implementation>()) {}
FRANKA::~FRANKA() {}

// ========== Public Interface Implementation ==========

bool FRANKA::init(RUT::TimePoint time0, const FRANKAConfig& config) {
    return impl_->initialize(time0, config);
}

bool FRANKA::getCartesian(RUT::Vector7d& pose_xyzq) {
    return impl_->getCartesian(pose_xyzq);
}

bool FRANKA::setCartesian(const RUT::Vector7d& pose_xyzq) {
    return impl_->setCartesian(pose_xyzq);
}   

bool FRANKA::getJoints(RUT::VectorXd& joints) {
    return impl_->getJoints(joints);
}

bool FRANKA::setJoints(const RUT::VectorXd& joints) {
    return impl_->setJoints(joints);
}

bool FRANKA::getTorques(RUT::VectorXd& torques) {
    return impl_->getTorques(torques);
}

bool FRANKA::setTorques(const RUT::VectorXd& torques) {
    return impl_->setTorques(torques);
}

bool FRANKA::getCartesianVelocity(RUT::Vector6d& velocity) {
    return impl_->getCartesianVelocity(velocity);
}

franka::RobotState FRANKA::readOnce() {
    return impl_->readOnce();
}

franka::RobotState FRANKA::update(const research_interface::robot::MotionGeneratorCommand* motion_command,
                                   const research_interface::robot::ControllerCommand* control_command) {
    return impl_->update(motion_command, control_command);
}

uint32_t FRANKA::startMotion(
    research_interface::robot::Move::ControllerMode controller_mode,
    research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
    const research_interface::robot::Move::Deviation& maximum_path_deviation,
    const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) {
    return impl_->startMotion(controller_mode, motion_generator_mode, maximum_path_deviation, maximum_goal_pose_deviation);
}

void FRANKA::finishMotion(
    uint32_t motion_id,
    const research_interface::robot::MotionGeneratorCommand* motion_command,
    const research_interface::robot::ControllerCommand* control_command) {  
    impl_->finishMotion(motion_id, motion_command, control_command);
}

void FRANKA::cancelMotion(uint32_t motion_id) {
    impl_->cancelMotion(motion_id);
}

void FRANKA::setJointImpedance(const std::array<double, 7>& K_theta) {
    impl_->setJointImpedance(K_theta);
}

void FRANKA::setCartesianImpedance(const std::array<double, 6>& K_x) {
    impl_->setCartesianImpedance(K_x);
}

void FRANKA::throwOnMotionError(const franka::RobotState& robot_state, uint32_t motion_id) {
    impl_->throwOnMotionError(robot_state, motion_id);
}

void FRANKA::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds_acceleration,
    const std::array<double, 7>& upper_torque_thresholds_acceleration,
    const std::array<double, 7>& lower_torque_thresholds_nominal,
    const std::array<double, 7>& upper_torque_thresholds_nominal,
    const std::array<double, 6>& lower_force_thresholds_acceleration,
    const std::array<double, 6>& upper_force_thresholds_acceleration,
    const std::array<double, 6>& lower_force_thresholds_nominal,
    const std::array<double, 6>& upper_force_thresholds_nominal) {
    
    impl_->setCollisionBehavior(lower_torque_thresholds_acceleration,
                                upper_torque_thresholds_acceleration,
                                lower_torque_thresholds_nominal,
                                upper_torque_thresholds_nominal,
                                lower_force_thresholds_acceleration,
                                upper_force_thresholds_acceleration,
                                lower_force_thresholds_nominal,
                                upper_force_thresholds_nominal);
}

void FRANKA::setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,
               const std::array<double, 9>& load_inertia) {
    impl_->setLoad(load_mass, F_x_Cload, load_inertia);
}

franka::Model FRANKA::loadModel() {
    return impl_->loadModel();
}

void FRANKA::finishCurrentMotion() {
    impl_->finishCurrentMotion();
}

bool FRANKA::getWrenchBaseOnTool(RUT::Vector6d& wrench) {
    return impl_->getWrenchBaseOnTool(wrench);
}

bool FRANKA::getWrenchTool(RUT::Vector6d& wrench) {
    return impl_->getWrenchTool(wrench);
}

bool FRANKA::getCurrentPose(RUT::Vector7d& pose_xyzq) {
    return impl_->getCurrentPose(pose_xyzq);
}

bool FRANKA::getCurrentWrenchTool(RUT::Vector6d& wrench) {
    return impl_->getCurrentWrenchTool(wrench);
}

franka::Duration FRANKA::getElapsedTime() {
    return impl_->getElapsedTime();
}

franka::RobotState FRANKA::getRobotState() {
    return impl_->getRobotState();
}

// ========== Low-Level Robot Interface Implementation ==========

franka::RobotState FRANKA::Implementation::readOnce() {
    return robot_impl->readOnce();
}

franka::RobotState FRANKA::Implementation::update(const research_interface::robot::MotionGeneratorCommand* motion_command,
                                                  const research_interface::robot::ControllerCommand* control_command) {
    return robot_impl->update(motion_command, control_command);
}

franka::RealtimeConfig FRANKA::Implementation::realtimeConfig() const noexcept {
    return robot_impl->realtimeConfig();
}

uint32_t FRANKA::Implementation::startMotion(
    research_interface::robot::Move::ControllerMode controller_mode,
    research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
    const research_interface::robot::Move::Deviation& maximum_path_deviation,
    const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) {
    
    try {
        // Reset cached commands for new motion session
        motion_command = research_interface::robot::MotionGeneratorCommand{};
        control_command = research_interface::robot::ControllerCommand{};
        motion_command.motion_generation_finished = false;

        motion_id = robot_impl->startMotion(controller_mode,
                                            motion_generator_mode,
                                            maximum_path_deviation,
                                            maximum_goal_pose_deviation);
        return motion_id;
    } catch (const std::exception& e) {
        std::cerr << "Error while starting motion: " << e.what() << std::endl;
        return 0; // Indicate failure
    }
}

void FRANKA::Implementation::finishMotion(
    uint32_t motion_id,
    const research_interface::robot::MotionGeneratorCommand* motion_command,
    const research_interface::robot::ControllerCommand* control_command) {
    try {
        std::cout << "Finishing motion with ID: " << motion_id << std::endl;
        robot_impl->finishMotion(motion_id, motion_command, control_command);
    } catch (const std::exception& e) {
        std::cerr << "Error while finishing motion: " << e.what() << std::endl;
    }
}

void FRANKA::Implementation::cancelMotion(uint32_t motion_id) {
    try {
        robot_impl->cancelMotion(motion_id);
    } catch (const std::exception& e) {
        std::cerr << "Error while canceling motion: " << e.what() << std::endl;
    }
}

void FRANKA::Implementation::setJointImpedance(const std::array<double, 7>& K_theta) {
    robot_impl->executeCommand<research_interface::robot::SetJointImpedance>(K_theta);
}

void FRANKA::Implementation::setCartesianImpedance(const std::array<double, 6>& K_x) {
    robot_impl->executeCommand<research_interface::robot::SetCartesianImpedance>(K_x);
}

void FRANKA::Implementation::throwOnMotionError(const franka::RobotState& robot_state, uint32_t motion_id) {
    robot_impl->throwOnMotionError(robot_state, motion_id);
}

void FRANKA::Implementation::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds_acceleration,
    const std::array<double, 7>& upper_torque_thresholds_acceleration,
    const std::array<double, 7>& lower_torque_thresholds_nominal,
    const std::array<double, 7>& upper_torque_thresholds_nominal,
    const std::array<double, 6>& lower_force_thresholds_acceleration,
    const std::array<double, 6>& upper_force_thresholds_acceleration,
    const std::array<double, 6>& lower_force_thresholds_nominal,
    const std::array<double, 6>& upper_force_thresholds_nominal) {
    
    robot_impl->executeCommand<research_interface::robot::SetCollisionBehavior>(
        lower_torque_thresholds_acceleration,
        upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal,
        upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration,
        upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal,
        upper_force_thresholds_nominal);
}

void FRANKA::Implementation::setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,
               const std::array<double, 9>& load_inertia) {
    robot_impl->executeCommand<research_interface::robot::SetLoad>(
        load_mass, F_x_Cload, load_inertia);
}

franka::Model FRANKA::Implementation::loadModel() {
    return robot_impl->loadModel();
}

void FRANKA::Implementation::finishCurrentMotion() {
    // Check if control command is empty (position/velocity control mode)
    if(control_command.tau_J_d.empty()) {
        // Set flag to indicate motion is finished
        motion_command.motion_generation_finished = true;
        // Fill motion command with current state to avoid issues
        motion_command.O_T_EE_c = robot_state.O_T_EE_c;
        motion_command.O_dP_EE_c = robot_state.O_dP_EE_c;

        // Send final update to robot with finished flag
        robot_state = update(&motion_command, nullptr);
        throwOnMotionError(robot_state, motion_id);
    }
    else {
        // Fill motion command joint velocity with 0 to stop motion
        motion_command.dq_c = {0, 0, 0, 0, 0, 0, 0};
        motion_command.motion_generation_finished = true;

        // Control command with the last commanded torques
        control_command.tau_J_d = robot_state.tau_J_d;

        // Send final update to robot with finished flag
        robot_state = update(&motion_command, &control_command);
        throwOnMotionError(robot_state, motion_id);
    }

    // Finish motion with current motion_id and commands
    finishMotion(motion_id, &motion_command, &control_command);
    std::cout << "Motion session finished." << std::endl;
}

// ========== State Getter Implementations ==========

bool FRANKA::Implementation::getJoints(RUT::VectorXd& joints) {
    try {
        robot_state = readOnce();
        // Map robot state joint positions to Eigen vector
        joints = Eigen::Map<const Eigen::VectorXd>(robot_state.q.data(), 7);
        return true;
    } catch (const franka::Exception& e) {
        std::cerr << "[Franka getJoints] libfranka error: " << e.what() << std::endl;
        return false;
    }
}

bool FRANKA::Implementation::getCartesian(RUT::Vector7d& pose_xyzq) {
    try {
        // Read current robot state
        robot_state = readOnce();
        RUT::Matrix4d M;

        // Convert from std::array<double,16> column-major to Eigen::Matrix4d
        std::copy(robot_state.O_T_EE.begin(), robot_state.O_T_EE.end(), M.data());

        // Convert from Eigen::Matrix4d to RUT::Vector7d pose
        RUT::SE32Pose(M, pose_xyzq);

        return true;
    } catch (const franka::Exception& e) {
        std::cerr << "[Franka getCartesian] libfranka error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[Franka getCartesian] std::exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[Franka getCartesian] unknown exception\n";
        return false;
    }
}

bool FRANKA::Implementation::getTorques(RUT::VectorXd& torques) {
    try {
        robot_state = readOnce();
        torques = Eigen::Map<const RUT::VectorXd>(robot_state.tau_J.data(), 7);
        return true;
    } catch (...) {
        return false;
    }
}

bool FRANKA::Implementation::getWrenchBaseOnTool(RUT::Vector6d& wrench) {
    try {
        robot_state = readOnce();
        wrench = Eigen::Map<const RUT::Vector6d>(robot_state.O_F_ext_hat_K.data());
        return true;
    } catch (...) {
        return false;
    }
}

bool FRANKA::Implementation::getWrenchTool(RUT::Vector6d& wrench) {
    try {
        robot_state = readOnce();
        wrench = Eigen::Map<const RUT::Vector6d>(robot_state.K_F_ext_hat_K.data());
        return true;
    } catch (...) {
        return false;
    }
}

bool FRANKA::Implementation::getCartesianVelocity(RUT::Vector6d& velocity) {
    try {
        // Read from cached state without calling readOnce() to avoid latency
        const auto& robot_state = this->robot_state;
        velocity = Eigen::Map<const RUT::Vector6d>(robot_state.O_dP_EE_c.data());
        return true;
    } catch (...) {
        return false;
    }
}

// ========== State Setter Implementations ==========

bool FRANKA::Implementation::setCartesian(const RUT::Vector7d& pose) {
    try {
        // Build transformation matrix from pose
        RUT::Vector3d position(pose[0], pose[1], pose[2]);
        RUT::Quaterniond quat(pose[3], pose[4], pose[5], pose[6]); // (w, x, y, z)
        RUT::Matrix3d rotation = quat.toRotationMatrix();

        RUT::Matrix4d M = RUT::Matrix4d::Identity();
        M.block<3,3>(0,0) = rotation;
        M.block<3,1>(0,3) = position;

        // Convert from Eigen::Matrix4d to std::array<double,16> column-major
        std::array<double, 16> O_T_EE_c{};
        std::copy(M.data(), M.data() + 16, O_T_EE_c.begin());

        // Apply filtering and rate limiting
        O_T_EE_c = franka::cartesianLowpassFilter(
            config.kDeltaT,
            O_T_EE_c,
            robot_state.O_T_EE_c,
            config.CutoffFrequency
        );

        O_T_EE_c = franka::limitRate(
            config.kMaxTranslationalVelocity,
            config.kMaxTranslationalAcceleration,
            config.kMaxTranslationalJerk,
            config.kMaxRotationalVelocity,
            config.kMaxRotationalAcceleration,
            config.kMaxRotationalJerk,
            O_T_EE_c,
            robot_state.O_T_EE_c,
            robot_state.O_dP_EE_c,
            robot_state.O_ddP_EE_c
        );

        // Send command and update robot state
        motion_command.O_T_EE_c = O_T_EE_c;
        robot_state = update(&motion_command, nullptr);
        throwOnMotionError(robot_state, motion_id);
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Motion error: " << e.what() << std::endl;
        return false;
    }
}

bool FRANKA::Implementation::setJoints(const RUT::VectorXd& joints) {
    if (joints.size() != 7) {
        return false;
    }

    try {
        motion_command.motion_generation_finished = false;
        motion_command.valid_elbow = false;

        std::array<double, 7> q_c{};
        std::copy(joints.data(), joints.data() + 7, q_c.begin());

        // Apply low-pass filtering
        for (size_t i = 0; i < 7; ++i) {
            q_c[i] = franka::lowpassFilter(config.kDeltaT,
                                            q_c[i],
                                            robot_state.q_d[i],
                                            franka::kDefaultCutoffFrequency);
        }
        
        // Apply rate limiting
        q_c = franka::limitRate(franka::kMaxJointVelocity,
                                franka::kMaxJointAcceleration,
                                franka::kMaxJointJerk,
                                q_c,
                                robot_state.q_d,
                                robot_state.dq_d,
                                robot_state.ddq_d);

        // Set motion command
        std::copy(q_c.begin(), q_c.end(), motion_command.q_c.begin());
        std::fill(motion_command.dq_c.begin(), motion_command.dq_c.end(), 0.0);

        robot_state = update(&motion_command, nullptr);
        throwOnMotionError(robot_state, motion_id);
        return true;
    } catch (const franka::Exception& e) {
        std::cerr << "[Franka setJoints] libfranka error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[Franka setJoints] std::exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[Franka setJoints] unknown exception" << std::endl;
        return false;
    }
}

bool FRANKA::Implementation::setTorques(const RUT::VectorXd& torques) {
    // Check size only in debug mode for performance
    #ifdef DEBUG
    if (torques.size() != 7) {
        return false;
    }
    #endif

    try {
        // Direct memory copy for performance
        std::memcpy(control_command.tau_J_d.data(), torques.data(), 7 * sizeof(double));

        // Zero velocity commands
        motion_command.dq_c = {0, 0, 0, 0, 0, 0, 0};
        
        // Apply low-pass filtering (unrolled for performance)
        control_command.tau_J_d[0] = franka::lowpassFilter(config.kDeltaT, control_command.tau_J_d[0], robot_state.tau_J_d[0], franka::kDefaultCutoffFrequency);
        control_command.tau_J_d[1] = franka::lowpassFilter(config.kDeltaT, control_command.tau_J_d[1], robot_state.tau_J_d[1], franka::kDefaultCutoffFrequency);
        control_command.tau_J_d[2] = franka::lowpassFilter(config.kDeltaT, control_command.tau_J_d[2], robot_state.tau_J_d[2], franka::kDefaultCutoffFrequency);
        control_command.tau_J_d[3] = franka::lowpassFilter(config.kDeltaT, control_command.tau_J_d[3], robot_state.tau_J_d[3], franka::kDefaultCutoffFrequency);
        control_command.tau_J_d[4] = franka::lowpassFilter(config.kDeltaT, control_command.tau_J_d[4], robot_state.tau_J_d[4], franka::kDefaultCutoffFrequency);
        control_command.tau_J_d[5] = franka::lowpassFilter(config.kDeltaT, control_command.tau_J_d[5], robot_state.tau_J_d[5], franka::kDefaultCutoffFrequency);
        control_command.tau_J_d[6] = franka::lowpassFilter(config.kDeltaT, control_command.tau_J_d[6], robot_state.tau_J_d[6], franka::kDefaultCutoffFrequency);
        
        // Apply rate limiting
        control_command.tau_J_d = franka::limitRate(
            franka::kMaxTorqueRate,
            control_command.tau_J_d,
            robot_state.tau_J_d
        );
        
        // Update robot state
        robot_state = update(&motion_command, &control_command);
        throwOnMotionError(robot_state, motion_id);

        return true;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return false;
    }
}

// ========== Helper Method Implementations ==========

bool FRANKA::Implementation::getCurrentPose(RUT::Vector7d& pose_xyzq) {
    try {
        const auto& T = robot_state.O_T_EE;   // column-major, 16 elements

        // Map to Eigen (column-major)
        Eigen::Matrix4d mat;
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                mat(r, c) = T[c * 4 + r];

        Eigen::Affine3d tf(mat);
        const Eigen::Vector3d p = tf.translation();
        const Eigen::Quaterniond q(tf.rotation());  // already normalized

        // Store as [x, y, z, qw, qx, qy, qz]
        pose_xyzq[0] = p.x();
        pose_xyzq[1] = p.y();
        pose_xyzq[2] = p.z();
        pose_xyzq[3] = q.w();
        pose_xyzq[4] = q.x();
        pose_xyzq[5] = q.y();
        pose_xyzq[6] = q.z();

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Franka getCurrentPose] std::exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[Franka getCurrentPose] unknown exception\n";
        return false;
    }
}

bool FRANKA::Implementation::getCurrentWrenchTool(RUT::Vector6d& wrench) {
    try {
        // Apply filtering to the wrench
        static RUT::Vector6d filtered_wrench = RUT::Vector6d::Zero();
        wrench = Eigen::Map<const RUT::Vector6d>(robot_state.K_F_ext_hat_K.data());

        for (size_t i = 0; i < 6; ++i) {
            filtered_wrench[i] = franka::lowpassFilter(config.kDeltaT,
                                            filtered_wrench[i],
                                            robot_state.K_F_ext_hat_K[i],
                                            franka::kDefaultCutoffFrequency);
        }
        
        filtered_wrench = Eigen::Map<const RUT::Vector6d>(robot_state.K_F_ext_hat_K.data());

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Franka getCurrentWrench] std::exception: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[Franka getCurrentWrench] unknown exception\n";
        return false;
    }
}

franka::Duration FRANKA::Implementation::getElapsedTime() {
    return robot_state.time;
}

franka::RobotState FRANKA::Implementation::getRobotState() {
    return robot_state;
}

