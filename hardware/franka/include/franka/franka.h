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
            RUT::Vector6d cartesian_impedance{};
            RUT::Vector3d deviation{10.0, 3.12, 2 * M_PI};

            RobotInterfaceConfig robot_interface_config{};
    
            bool deserialize(const YAML::Node& node) {
                try {
                    robot_ip = node["robot_ip"].as<std::string>();
                    log_size = node["log_size"].as<double>();
                    robot_interface_config.deserialize(node["robot_interface_config"]);
                    tcp_mass = node["tcp_mass"].as<double>();
                    tcp_inertia = node["tcp_inertia"].as<double>();
                    cartesian_impedance = RUT::deserialize_vector<RUT::Vector6d>(node["cartesian_impedance"]);
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

        //not robot_interfaces functions

        //get Wrench Base on Tool 
        bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
        //get Wrench Tool
        bool getWrenchTool(RUT::Vector6d& wrench);
        bool getTorques(RUT::VectorXd& torques);
        bool setTorques(const RUT::VectorXd& torques);

        static constexpr research_interface::robot::Move::Deviation kDefaultDeviation{10.0, 3.12,
                                                                                2 * M_PI};

        //readOnce reads the state of the robot once
        franka::RobotState readOnce();
        //update updates the state of the robot with the given motion command and control command
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
            
    private:
        // crear un puntero a la clase Robot::Impl (Pimpl idiom)
        struct Implementation;
        std::unique_ptr<Implementation> impl_;
};

#endif // _FRANKA_HEADER_