#include "franka/franka.h"
#include <chrono>

#include <Eigen/Dense>
#include "robot_impl.h"
#include "network.h"
#include <iostream>
  
struct FRANKA::Implementation {
    //puntero a la implementacion de la clase Robot
    std::unique_ptr<franka::Robot::Impl> robot_impl;
    //id del motion
    uint32_t motion_id;
    //configuracion del robot desde yaml
    FRANKA::FRANKAConfig config{};

    //constructor que recibe la configuracion del robot
    Implementation(const FRANKA::FRANKAConfig& config) 
    {

    std::cout << "ip: " << config.robot_ip << std::endl;
    //crea un objeto Network con la ip del robot y el tamaño del log
    //instancia la implementacion del robot
    std::unique_ptr<franka::Network> network;
    try {
        std::cout << "[DEBUG] Attempting to create Network..." << std::endl;

        network = std::make_unique<franka::Network>(
            config.robot_ip,
            research_interface::robot::kCommandPort);

        std::cout << "[DEBUG] Network created successfully!" << std::endl;

        } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception while creating Network: " << e.what() << std::endl;
        return;
        }
    // Create Robot::Impl using your Network
    robot_impl = std::make_unique<franka::Robot::Impl>(
        std::move(network),
        config.log_size,
        franka::RealtimeConfig::kEnforce);
        this->config = config;   
    }
    
    // Destructor que libera los recursos de la implementacion del robot
    ~Implementation() {}

    // metodos para obtener estado del robot
    bool getCartesian(RUT::Vector7d& pose);
    bool getJoints(RUT::VectorXd& joints);
    bool getTorques(RUT::VectorXd& torques);
    bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
    bool getWrenchTool(RUT::Vector6d& wrench);

    // metodos para setear estado del robot
    bool setCartesian(const RUT::Vector7d& pose);
    bool setJoints(const RUT::VectorXd& joints);
    bool setTorques(const RUT::VectorXd& torques);

    //metodos de robot_impl.h
    franka::RobotState readOnce();
    franka::RobotState update(const research_interface::robot::MotionGeneratorCommand* motion_command,
                              const research_interface::robot::ControllerCommand* control_command);
    franka::RealtimeConfig realtimeConfig() const noexcept;
    uint32_t startMotion(research_interface::robot::Move::ControllerMode controller_mode,
                         research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
                         const research_interface::robot::Move::Deviation& maximum_path_deviation,
                         const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation);
    void finishMotion( uint32_t motion_id,
                      const research_interface::robot::MotionGeneratorCommand* motion_command,
                      const research_interface::robot::ControllerCommand* control_command);
    void cancelMotion(uint32_t motion_id);

    void throwOnMotionError(const franka::RobotState& robot_state, uint32_t motion_id);

    //funciones para setear impedancia en el robot (falta setCollisionBehavior)
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
    
};

template <typename T, size_t N>
inline void checkFinite(const std::array<T, N>& array) {
  if (!std::all_of(array.begin(), array.end(), [](double d) { return std::isfinite(d); })) {
    throw std::invalid_argument("Commanding value is infinite or NaN.");
  }
}

// Constructor que inicializa la implementacion del robot con la configuracion del struct FRANKAConfig
FRANKA::FRANKA(const FRANKAConfig& config){
    std::cout << "inside FRANKA constructor" << std::endl;
    impl_ = std::make_unique<Implementation>(config);
}
FRANKA::~FRANKA() {}

//funciones de llamada a impl de la clase FRANKA
bool FRANKA::getCartesian(RUT::Vector7d& pose) {
    return impl_->getCartesian(pose);
}
bool FRANKA::setCartesian(const RUT::Vector7d& pose) {
    return impl_->setCartesian(pose);
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
bool FRANKA::getWrenchBaseOnTool(RUT::Vector6d& wrench) {
    return impl_->getWrenchBaseOnTool(wrench);
}
bool FRANKA::getWrenchTool(RUT::Vector6d& wrench) {
    return impl_->getWrenchTool(wrench);
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

    // Implement the finish motion logic here
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


//funciones de llamada a los metodos de robot_impl.h
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
        return robot_impl->startMotion(controller_mode, motion_generator_mode, maximum_path_deviation, maximum_goal_pose_deviation);
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


bool FRANKA::Implementation::getJoints(RUT::VectorXd& joints) {
    try {
        franka::RobotState state = readOnce();
        joints = Eigen::Map<const Eigen::VectorXd>(state.q.data(), 7);
        return true;
    } catch (...) {
        return false;
    }
}

bool FRANKA::Implementation::getCartesian(RUT::Vector7d& pose) {
    try {
        franka::RobotState state = readOnce();
        pose = Eigen::Map<const RUT::Vector7d>(state.O_T_EE.data(), 7);
        return true;
    } catch (...) {
        return false;
    }
}

bool FRANKA::Implementation::getTorques(RUT::VectorXd& torques) {
    try {
        franka::RobotState state = readOnce();
        torques = Eigen::Map<const RUT::VectorXd>(state.tau_J.data(), 7);
        return true;
    } catch (...) {
        return false;
    }
}

bool FRANKA::Implementation::getWrenchBaseOnTool(RUT::Vector6d& wrench) {
    try {
        franka::RobotState state = readOnce();
        wrench = Eigen::Map<const RUT::Vector6d>(state.O_F_ext_hat_K.data(), 6);
        return true;
    } catch (...) {
        return false;
    }
}

// Todo: AHORA ESTA SOBRE FRAME BASE, SE SUPONE QUE ES SOBRE FRAME TOOL
bool FRANKA::Implementation::getWrenchTool(RUT::Vector6d& wrench) {
    try {
        franka::RobotState state = readOnce();
        wrench = Eigen::Map<const RUT::Vector6d>(state.O_F_ext_hat_K.data(), 6);
        return true;
    } catch (...) {
        return false;
    }
}

bool FRANKA::Implementation::setCartesian(const RUT::Vector7d& pose) {
    if (pose.size() != 16) return false;  // Optional check

    try {
        research_interface::robot::MotionGeneratorCommand command;
        std::copy(pose.data(), pose.data() + 16, command.O_T_EE_c.begin());
        command.valid_elbow = true;
        command.motion_generation_finished = false;
        robot_impl->update(&command, nullptr);
        return true;
    } catch (...) {
        return false;
    }
}

bool FRANKA::Implementation::setJoints(const RUT::VectorXd& joints) {
    if (joints.size() != 7) {
        return false;
    }

    try {
        research_interface::robot::MotionGeneratorCommand command;
        std::copy(joints.data(), joints.data() + 7, command.q_c.begin());
        command.valid_elbow = true;
        command.motion_generation_finished = false;
        robot_impl->update(&command, nullptr);
        return true;
    } catch (...) {
        return false;
    }
}


bool FRANKA::Implementation::setTorques(const RUT::VectorXd& torques) {

    if (torques.size() != 7) {
        return false;
    }

    try {
        research_interface::robot::ControllerCommand command;
        std::copy(torques.data(), torques.data() + 7, command.tau_J_d.begin());
        robot_impl->update(nullptr, &command);
        return true;
    } catch (...) {
        return false;
    }
}
