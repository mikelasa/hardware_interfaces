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
#include <chrono>

#include "hardware_interfaces/robot_interfaces.h"

//include franka headers
#include "robot_impl.h"

class FRANKA : public RobotInterfaces {

    public:
        struct FRANKAConfig {
            std::string robot_ip{};
            double log_size{1000};
            double tcp_mass{0.0};
            double tcp_inertia{0.0};
            RUT::Vector3d deviation{10.0, 3.12, 2 * M_PI};

            RobotInterfaceConfig robot_interface_config{};
    
            bool deserialize(const YAML::Node& node) {
                try {
                    robot_ip = node["robot_ip"].as<std::string>();
                    log_size = node["log_size"].as<double>();
                    robot_interface_config.deserialize(node["robot_interface_config"]);
                    tcp_mass = node["tcp_mass"].as<double>();
                    tcp_inertia = node["tcp_inertia"].as<double>();
                    deviation = RUT::deserialize_vector<RUT::Vector3d>(node["deviation"]);
                } catch (const std::exception& e) {
                    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
                    return false;
                }
                return true;
            }
        };

        FRANKA(const FRANKAConfig& config);
        virtual ~FRANKA();

        static constexpr research_interface::robot::Move::Deviation kDefaultDeviation{10.0, 3.12, 2 * M_PI};

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
        bool getCartesian(RUT::Vector7d& pose) override;
        bool setCartesian(const RUT::Vector7d& pose) override;
        bool getJoints(RUT::VectorXd& joints) override;
        bool setJoints(const RUT::VectorXd& joints) override;
        bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
        bool getWrenchTool(RUT::Vector6d& wrench);
        bool getTorques(RUT::VectorXd& torques);
        bool setTorques(const RUT::VectorXd& torques);

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

    private:
        // crear un puntero a la clase Robot::Impl (Pimpl idiom)
        struct Implementation;
        std::unique_ptr<Implementation> impl_;
};

#endif // _FRANKA_HEADER_