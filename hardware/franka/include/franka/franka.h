/**
 * URRTDE: wrapper around ur franka implementation
 * https://sdurobotics.gitlab.io/ur_rtde/index.html
 *
 * Author:
 *      Mikel Lasa <mlasa@mondragon.edu>
 */

#ifndef _FRANKA_HEADER_
#define _FRANKA_HEADER_

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <array>
#include <chrono>

#include "hardware_interfaces/robot_interfaces.h"

//include franka headers
#include "robot_impl.h"
#include "model.h"

class Model;

class FRANKA : public RobotInterfaces {

    public:
        struct FRANKAConfig {
            std::string robot_ip{};
            double log_size{1000};
            double tcp_mass{0.1};
            std::array<double, 3> fx_c_load{{0.0, 0.0, 0.0}}; // default origin
            std::array<double, 9> tcp_inertia{{0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0}}; // default inertia
            RUT::Vector3d deviation{10.0, 3.12, 2 * M_PI};
            double kDeltaT{1e-3}; // Time step for filtering and rate limiting
            double CutoffFrequency{100}; // Cutoff frequency for low-pass filter
            std::string realtime_config{"enforce"}; // "ignore" or "enforce"
            std::string controller_mode{"external_controller"}; // "joint_impedance", "cartesian_impedance", "external_controller"
            std::string motion_generator_mode{"joint_velocity"}; // "joint_position", "joint_velocity", "cartesian_position", "cartesian_velocity"
            std::array<double, 7> setJointImpedance{{0, 0, 0, 0, 0, 0, 0}};
            std::array<double, 6> setCartesianImpedance{{0, 0, 0, 0, 0, 0}};
            double kMaxTranslationalVelocity{1.96};
            double kMaxTranslationalAcceleration{12.99};
            double kMaxTranslationalJerk{12500};
            double kMaxRotationalVelocity{2.424};
            double kMaxRotationalAcceleration{24.999};
            double kMaxRotationalJerk{12500};

            RobotInterfaceConfig robot_interface_config{};

            template <typename T, std::size_t N>
            static std::array<T, N> deserialize_array(const YAML::Node& node) {
                if (!node || !node.IsSequence()) {
                    throw std::invalid_argument("Expected a YAML sequence when parsing std::array");
                }
                if (node.size() != N) {
                    throw std::invalid_argument("YAML sequence size does not match std::array length");
                }
                std::array<T, N> data{};
                for (std::size_t i = 0; i < N; ++i) {
                    data[i] = node[i].as<T>();
                }
                return data;
            }
    
            bool deserialize(const YAML::Node& node) {
                try {
                    robot_ip = node["robot_ip"].as<std::string>();
                    log_size = node["log_size"].as<double>();
                    robot_interface_config.deserialize(node["robot_interface_config"]);
                    tcp_mass = node["tcp_mass"].as<double>();
                    fx_c_load = deserialize_array<double, 3>(node["fx_c_load"]);
                    tcp_inertia = deserialize_array<double, 9>(node["tcp_inertia"]);
                    deviation = RUT::deserialize_vector<RUT::Vector3d>(node["deviation"]);
                    kDeltaT = node["kDeltaT"].as<double>();
                    CutoffFrequency = node["CutoffFrequency"].as<double>();
                    realtime_config = node["realtime_config"].as<std::string>();
                    controller_mode = node["controller_mode"].as<std::string>();
                    motion_generator_mode = node["motion_generator_mode"].as<std::string>();
                    setJointImpedance = deserialize_array<double, 7>(node["setJointImpedance"]);
                    setCartesianImpedance = deserialize_array<double, 6>(node["setCartesianImpedance"]);
                    kMaxTranslationalVelocity = node["kMaxTranslationalVelocity"].as<double>();
                    kMaxTranslationalAcceleration = node["kMaxTranslationalAcceleration"].as<double>();
                    kMaxTranslationalJerk = node["kMaxTranslationalJerk"].as<double>();
                    kMaxRotationalVelocity = node["kMaxRotationalVelocity"].as<double>();
                    kMaxRotationalAcceleration = node["kMaxRotationalAcceleration"].as<double>();
                    kMaxRotationalJerk = node["kMaxRotationalJerk"].as<double>();
                } catch (const std::exception& e) {
                    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
                    return false;
                }
                return true;
            }
        };

        FRANKA();
        ~FRANKA();

        /**
         * Initialize socket communication. Create a thread to run the 500Hz
         * communication with URe.
         *
         * @param[in]  time0    Start time. Time will count from this number.
         * @param[in]  config   controller configs.
         *
         * @return     True if success.
         */
        bool init(RUT::TimePoint time0, const FRANKAConfig& config);

        /*
            *get Cartesian pose of the robot tool. Distances are in mm.
            * @param      pose  The Cartesian pose. [x y z qw qx qy qz]
            *get Cartesian velocity of the robot tool. Distances are in mm/s.
            * @param      velocity  The Cartesian velocity. [vx vy vz wx wy wz]
            *set Cartesian pose of the robot tool. Distances are in mm.
            * @param[in]  pose  The Cartesian pose. [x y z qw qx qy qz]
            *get joint angles in rad.
            * @param      joints  The joints.
            * set joint angles in rad.
            * @param[in]  joints  The joints.
        */
        bool getCartesian(RUT::Vector7d& pose_xyzq) override;
        bool setCartesian(const RUT::Vector7d& pose_xyzq) override;
        bool getJoints(RUT::VectorXd& joints) override;
        bool setJoints(const RUT::VectorXd& joints) override;
        bool getTorques(RUT::VectorXd& torques);
        bool setTorques(const RUT::VectorXd& torques);
        bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
        bool getWrenchTool(RUT::Vector6d& wrench);
        
        /* helpers to expose internal robot_state and get current pose and wrench without using readOnce()
        */
        bool getCurrentPose(RUT::Vector7d& pose_xyzq);
        bool getCurrentWrenchTool(RUT::Vector6d& wrench);
        franka::Duration getElapsedTime();
        franka::RobotState getRobotState();

        /* funciones de robot_impl.h 
        * readOnce lee el estado del robot una vez
        * update actualiza el estado del robot con el comando de generador de movimiento y el comando de controlador
        * startMotion inicia un movimiento del robot, devuelve el id del movimiento
        * finishMotion termina un movimiento del robot
        * cancelMotion cancela un movimiento del robot
        * throwOnMotionError lanza una excepcion si hay un error en el movimiento del robot
        * realtimeConfig devuelve la configuracion de tiempo real del robot
        */

        franka::RobotState readOnce();
        franka::RobotState update(const research_interface::robot::MotionGeneratorCommand* motion_command,
                                   const research_interface::robot::ControllerCommand* control_command);
        uint32_t startMotion(research_interface::robot::Move::ControllerMode controller_mode,
                         research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
                         const research_interface::robot::Move::Deviation& maximum_path_deviation,
                         const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation);
        void finishMotion( uint32_t motion_id,
                        const research_interface::robot::MotionGeneratorCommand* motion_command,
                        const research_interface::robot::ControllerCommand* control_command);
        void cancelMotion(uint32_t motion_id);
        void throwOnMotionError(const franka::RobotState& robot_state, uint32_t motion_id);

        // functions to start different motion sessions: cartesian, joint, impedance...
        bool startCartesianMotion(
        research_interface::robot::Move::ControllerMode controller_mode,
        research_interface::robot::Move::MotionGeneratorMode motion_generator_mode
        );

        void finishCurrentMotion();

        /*funciones para setear comportamientos en el robot
        * setJointImpedance setea la impedancia en las juntas del robot
        * setCartesianImpedance setea la impedancia en el espacio cartesiano del robot
        * setCollisionBehavior setea el comportamiento de colision del robot
        */
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

        //load model
        franka::Model loadModel();

        

    private:
        // crear un puntero a la clase Robot::Impl (Pimpl idiom)
        struct Implementation;
        std::unique_ptr<Implementation> impl_;
};

#endif // _FRANKA_HEADER_
