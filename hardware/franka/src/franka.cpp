#include "franka/franka.h"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include "lowpass_filter.h"
#include "rate_limiting.h"

#include "robot_impl.h"
#include "network.h"

template <typename T, size_t N>
inline void checkFinite(const std::array<T, N>& array) {
  if (!std::all_of(array.begin(), array.end(), [](double d) { return std::isfinite(d); })) {
    throw std::invalid_argument("Commanding value is infinite or NaN.");
  }
}
  
struct FRANKA::Implementation {
    //puntero a la implementacion de la clase Robot
    std::unique_ptr<franka::Robot::Impl> robot_impl;
    //id del motion
    uint32_t motion_id{0};
    // Último estado del robot
    franka::RobotState robot_state;
    //configuracion del robot desde yaml
    FRANKA::FRANKAConfig config{};
    // Persistent command objects
    research_interface::robot::MotionGeneratorCommand motion_command{};

    //constructor que recibe la configuracion del robot
    Implementation(const FRANKA::FRANKAConfig& config) 
    {

        std::cout << "ip: " << config.robot_ip << std::endl;
        //crea un objeto Network con la ip del robot y el tamaño del log
        //instancia la implementacion del robot
        std::unique_ptr<franka::Network> network;
        try {
            std::cout << "Attempting to create Network..." << std::endl;

            network = std::make_unique<franka::Network>(
                config.robot_ip,
                research_interface::robot::kCommandPort);

            std::cout << "Network created successfully!" << std::endl;

            } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception while creating Network: " << e.what() << std::endl;
            return;
            }
        
        // Set realtime configuration
        franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
        if (config.realtime_config == "ignore") {
            rt_config = franka::RealtimeConfig::kIgnore;
        } else if (config.realtime_config == "enforce") {
            rt_config = franka::RealtimeConfig::kEnforce;
        } 

        // set controller mode and motion generator mode
        research_interface::robot::Move::ControllerMode cm_config = research_interface::robot::Move::ControllerMode::kCartesianImpedance;
        research_interface::robot::Move::MotionGeneratorMode mg_config = research_interface::robot::Move::MotionGeneratorMode::kCartesianPosition;
        if (config.controller_mode == "joint_impedance") {
            cm_config = research_interface::robot::Move::ControllerMode::kJointImpedance;
        } else if (config.controller_mode == "cartesian_impedance") {
            cm_config = research_interface::robot::Move::ControllerMode::kCartesianImpedance;
        } else if (config.controller_mode == "external_controller") {
            cm_config = research_interface::robot::Move::ControllerMode::kExternalController;
        }
        if (config.motion_generator_mode == "joint_position") {
            mg_config = research_interface::robot::Move::MotionGeneratorMode::kJointPosition;
        } else if (config.motion_generator_mode == "joint_velocity") {
            mg_config = research_interface::robot::Move::MotionGeneratorMode::kJointVelocity;
        } else if (config.motion_generator_mode == "cartesian_position") {
            mg_config = research_interface::robot::Move::MotionGeneratorMode::kCartesianPosition;
        } else if (config.motion_generator_mode == "cartesian_velocity") {
            mg_config = research_interface::robot::Move::MotionGeneratorMode::kCartesianVelocity;
        }

        // Create Robot::Impl using your Network
        robot_impl = std::make_unique<franka::Robot::Impl>(
            std::move(network),
            config.log_size,
            rt_config);
            this->config = config;   
    }
    
    // Destructor que libera los recursos de la implementacion del robot
    ~Implementation() {}

    // helpers para exponer el estado interno robot_state y obtener pose y wrench actuales sin usar readOnce()
    bool getCurrentPose(RUT::Vector7d& pose_xyzq);
    bool getCurrentWrenchTool(RUT::Vector6d& wrench);
    franka::Duration getElapsedTime();

    // metodos para obtener estado del robot
    bool getCartesian(RUT::Vector7d& pose_xyzq);
    bool getJoints(RUT::VectorXd& joints);
    bool getTorques(RUT::VectorXd& torques);
    bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
    bool getWrenchTool(RUT::Vector6d& wrench);

    // metodos para setear estado del robot
    bool setCartesian(const RUT::Vector7d& pose_xyzq);
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
    void setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
               const std::array<double, 9>& load_inertia);

    //funcion para cargar el modelo del robot
    franka::Model loadModel();

    void finishCurrentMotion();
    
};

// Constructor que inicializa la implementacion del robot con la configuracion del struct FRANKAConfig
FRANKA::FRANKA(const FRANKAConfig& config){
    std::cout << "inside FRANKA constructor" << std::endl;
    impl_ = std::make_unique<Implementation>(config);
}
FRANKA::~FRANKA() {}

//funciones de llamada a impl de la clase FRANKA
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
void FRANKA::setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
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
        // Reset cached commands for the new motion session
        motion_command = research_interface::robot::MotionGeneratorCommand{};
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
    const research_interface::robot::ControllerCommand* control_command) 
    {
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
               const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
               const std::array<double, 9>& load_inertia) {
    // Create the request and execute the command
    robot_impl->executeCommand<research_interface::robot::SetLoad>(
        load_mass, F_x_Cload, load_inertia);
}

franka::Model FRANKA::Implementation::loadModel() {
    return robot_impl->loadModel();
}

void FRANKA::Implementation::finishCurrentMotion() {


    // Set flag to indicate motion is finished
    motion_command.motion_generation_finished = true;
    // Fill motion_command with 0 to avoid issues
    motion_command.O_T_EE_c = robot_state.O_T_EE_c;
    motion_command.O_dP_EE_c = robot_state.O_dP_EE_c;

    // Send final update to robot with finished flag
    robot_state = update(&motion_command, nullptr);

    // Now call finishMotion with current motion_id and motion_command, no control_command
    finishMotion(motion_id, &motion_command, nullptr);
    std::cout << "Motion session finished." << std::endl;

}

// IMPLEMENTACION DE FUNCIONES
bool FRANKA::Implementation::getJoints(RUT::VectorXd& joints) {
    try {
        robot_state = readOnce();
        // pass state.q 
        joints = Eigen::Map<const Eigen::VectorXd>(robot_state.q.data(), 7);
        return true;
    } catch (const franka::Exception& e) {
    std::cerr << "[Franka getJoints] libfranka error: " << e.what() << std::endl;
    return false;
    }
}

bool FRANKA::Implementation::getCartesian(RUT::Vector7d& pose_xyzq) {
  try {

    //via RUT SE32pose
    robot_state = readOnce();
    RUT::Matrix4d M;

    //convert from std::array<double,16> column-major to Eigen::Matrix4d
    std::copy(robot_state.O_T_EE.begin(), robot_state.O_T_EE.end(), M.data());

    //convert from Eigen::Matrix4d to RUT::Vector7d pose
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

bool FRANKA::Implementation::setCartesian(const RUT::Vector7d& pose) {
  try {

        
        // Manually build transformation matrix from pose
        RUT::Vector3d position(pose[0], pose[1], pose[2]);
        RUT::Quaterniond quat(pose[3], pose[4], pose[5], pose[6]); // (w, x, y, z)
        RUT::Matrix3d rotation = quat.toRotationMatrix();

        RUT::Matrix4d M = RUT::Matrix4d::Identity();
        M.block<3,3>(0,0) = rotation;
        M.block<3,1>(0,3) = position;

        // convert from Eigen::Matrix4d to std::array<double,16> column-major
        std::array<double, 16> O_T_EE_c{};
        std::copy(M.data(), M.data() + 16, O_T_EE_c.begin());

        // Filtering and rate limiting
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
        motion_command.valid_elbow = false;  // or true if you populate elbow_c

        std::array<double, 7> q_c{};
        std::copy(joints.data(), joints.data() + 7, q_c.begin());

        //std::cout << "before qc: " << Eigen::Map<const RUT::VectorXd>(q_c.data(), 7).transpose() << std::endl;
        //print robot state
        //std::cout << "robot_state.q_d: " << Eigen::Map<const RUT::VectorXd>(robot_state.q_d.data(), 7).transpose() << std::endl;
        for (size_t i = 0; i < 7; ++i) {
        q_c[i] = franka::lowpassFilter(config.kDeltaT,
                                        q_c[i],
                                        robot_state.q_d[i],
                                        franka::kDefaultCutoffFrequency);
        }
        
        //std::cout << "after qc: " << Eigen::Map<const RUT::VectorXd>(q_c.data(), 7).transpose() << std::endl;
        q_c = franka::limitRate(franka::kMaxJointVelocity,
                                franka::kMaxJointAcceleration,
                                franka::kMaxJointJerk,
                                q_c,
                                robot_state.q_d,
                                robot_state.dq_d,
                                robot_state.ddq_d);

        //std::cout << "after rate limiting: " << Eigen::Map<const RUT::VectorXd>(q_c.data(), 7).transpose() << std::endl;

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

        // IMPORTANT: store as [x, y, z, qx, qy, qz, qw]
        pose_xyzq[0] = p.x();
        pose_xyzq[1] = p.y();
        pose_xyzq[2] = p.z();
        pose_xyzq[3] = q.x();
        pose_xyzq[4] = q.y();
        pose_xyzq[5] = q.z();
        pose_xyzq[6] = q.w();

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
        wrench = Eigen::Map<const RUT::Vector6d>(robot_state.K_F_ext_hat_K.data());
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