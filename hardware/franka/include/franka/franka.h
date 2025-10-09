/**
 * FRANKA: Wrapper around Franka robot implementation
 * 
 * This class provides a high-level interface for controlling Franka robots,
 * implementing the RobotInterfaces base class for standardized robot control.
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

// Include Franka headers
#include "robot_impl.h"
#include "model.h"

class Model;

class FRANKA : public RobotInterfaces {

    public:
        /**
         * Configuration structure for FRANKA robot initialization
         * Contains all necessary parameters for robot setup and control
         */
        struct FRANKAConfig {
            std::string robot_ip{};                                 // Robot IP address
            double log_size{1000};                                  // Log buffer size
            double tcp_mass{0.1};                                   // Tool Center Point mass (kg)
            std::array<double, 3> fx_c_load{{0.0, 0.0, 0.0}};      // Load center of mass position
            std::array<double, 9> tcp_inertia{{0.0, 0.0, 0.0,      // Tool inertia matrix
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0}};
            RUT::Vector3d deviation{10.0, 3.12, 2 * M_PI};         // Maximum path deviation [trans, rot, elbow]
            double kDeltaT{1e-3};                                   // Time step for filtering and rate limiting
            double CutoffFrequency{100};                            // Cutoff frequency for low-pass filter
            std::string realtime_config{"enforce"};                 // Realtime config: "ignore" or "enforce"
            std::string controller_mode{"external_controller"};     // Controller mode options
            std::string motion_generator_mode{"joint_velocity"};    // Motion generator mode options
            std::array<double, 7> setJointImpedance{{0, 0, 0, 0, 0, 0, 0}};        // Joint impedance values
            std::array<double, 6> setCartesianImpedance{{0, 0, 0, 0, 0, 0}};       // Cartesian impedance values
            
            // Motion limits
            double kMaxTranslationalVelocity{1.96};                 // Max translational velocity (m/s)
            double kMaxTranslationalAcceleration{12.99};            // Max translational acceleration (m/s²)
            double kMaxTranslationalJerk{12500};                    // Max translational jerk (m/s³)
            double kMaxRotationalVelocity{2.424};                   // Max rotational velocity (rad/s)
            double kMaxRotationalAcceleration{24.999};              // Max rotational acceleration (rad/s²)
            double kMaxRotationalJerk{12500};                       // Max rotational jerk (rad/s³)

            RobotInterfaceConfig robot_interface_config{};

            /**
             * Helper function to deserialize YAML arrays into std::array
             */
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
    
            /**
             * Deserialize configuration from YAML node
             * @param node YAML node containing configuration parameters
             * @return true if deserialization successful, false otherwise
             */
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
         * Initialize robot communication and create control thread
         * 
         * @param time0  Start time reference point
         * @param config Robot configuration parameters
         * @return true if initialization successful, false otherwise
         */
        bool init(RUT::TimePoint time0, const FRANKAConfig& config);

        // ========== RobotInterfaces Implementation ==========
        
        /**
         * Get current Cartesian pose of the robot tool
         * @param pose_xyzq Cartesian pose [x, y, z, qw, qx, qy, qz] (distances in mm)
         * @return true if successful
         */
        bool getCartesian(RUT::Vector7d& pose_xyzq) override;
        
        /**
         * Set target Cartesian pose of the robot tool
         * @param pose_xyzq Target Cartesian pose [x, y, z, qw, qx, qy, qz] (distances in mm)
         * @return true if successful
         */
        bool setCartesian(const RUT::Vector7d& pose_xyzq) override;
        
        /**
         * Get current joint angles
         * @param joints Joint angles in radians
         * @return true if successful
         */
        bool getJoints(RUT::VectorXd& joints) override;
        
        /**
         * Set target joint angles
         * @param joints Target joint angles in radians
         * @return true if successful
         */
        bool setJoints(const RUT::VectorXd& joints) override;
        
        /**
         * Get current Cartesian velocity of the robot tool
         * @param velocity Cartesian velocity [vx, vy, vz, wx, wy, wz] (mm/s and rad/s)
         * @return true if successful
         */
        bool getCartesianVelocity(RUT::Vector6d& velocity) override;

        // ========== FRANKA Specific Methods ==========
        
        /**
         * Get current joint torques
         * @param torques Joint torques in Nm
         * @return true if successful
         */
        bool getTorques(RUT::VectorXd& torques);
        
        /**
         * Set target joint torques
         * @param torques Target joint torques in Nm
         * @return true if successful
         */
        bool setTorques(const RUT::VectorXd& torques);
        
        /**
         * Get wrench at tool frame expressed in base frame
         * @param wrench Wrench [fx, fy, fz, mx, my, mz]
         * @return true if successful
         */
        bool getWrenchBaseOnTool(RUT::Vector6d& wrench);
        
        /**
         * Get wrench at tool frame
         * @param wrench Wrench [fx, fy, fz, mx, my, mz]
         * @return true if successful
         */
        bool getWrenchTool(RUT::Vector6d& wrench);
        
        // ========== Helper Methods ==========
        
        /**
         * Get current pose without using readOnce()
         * @param pose_xyzq Current Cartesian pose [x, y, z, qw, qx, qy, qz]
         * @return true if successful
         */
        bool getCurrentPose(RUT::Vector7d& pose_xyzq);
        
        /**
         * Get current wrench at tool without using readOnce()
         * @param wrench Current wrench [fx, fy, fz, mx, my, mz]
         * @return true if successful
         */
        bool getCurrentWrenchTool(RUT::Vector6d& wrench);
        
        /**
         * Get elapsed time since robot initialization
         * @return Elapsed time duration
         */
        franka::Duration getElapsedTime();
        
        /**
         * Get current robot state
         * @return Current robot state
         */
        franka::RobotState getRobotState();

        // ========== Low-Level Robot Control ==========
        
        /**
         * Read robot state once
         * @return Current robot state
         */
        franka::RobotState readOnce();
        
        /**
         * Update robot state with motion and control commands
         * @param motion_command Motion generator command
         * @param control_command Controller command
         * @return Updated robot state
         */
        franka::RobotState update(const research_interface::robot::MotionGeneratorCommand* motion_command,
                                   const research_interface::robot::ControllerCommand* control_command);
        
        /**
         * Start a new motion with specified modes and deviations
         * @param controller_mode Controller mode for the motion
         * @param motion_generator_mode Motion generator mode
         * @param maximum_path_deviation Maximum allowed path deviation
         * @param maximum_goal_pose_deviation Maximum allowed goal pose deviation
         * @return Motion ID for tracking
         */
        uint32_t startMotion(research_interface::robot::Move::ControllerMode controller_mode,
                         research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
                         const research_interface::robot::Move::Deviation& maximum_path_deviation,
                         const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation);
        
        /**
         * Finish a motion with final commands
         * @param motion_id ID of the motion to finish
         * @param motion_command Final motion command
         * @param control_command Final control command
         */
        void finishMotion(uint32_t motion_id,
                        const research_interface::robot::MotionGeneratorCommand* motion_command,
                        const research_interface::robot::ControllerCommand* control_command);
        
        /**
         * Cancel an active motion
         * @param motion_id ID of the motion to cancel
         */
        void cancelMotion(uint32_t motion_id);
        
        /**
         * Check for motion errors and throw exception if found
         * @param robot_state Current robot state to check
         * @param motion_id ID of the motion to check
         */
        void throwOnMotionError(const franka::RobotState& robot_state, uint32_t motion_id);

        // ========== Motion Session Management ==========
        
        /**
         * Start a Cartesian motion session
         * @param controller_mode Controller mode for the motion
         * @param motion_generator_mode Motion generator mode
         * @return true if motion started successfully
         */
        bool startCartesianMotion(
            research_interface::robot::Move::ControllerMode controller_mode,
            research_interface::robot::Move::MotionGeneratorMode motion_generator_mode
        );

        /**
         * Finish the current active motion session
         */
        void finishCurrentMotion();

        // ========== Robot Behavior Configuration ==========
        
        /**
         * Set joint impedance parameters
         * @param K_theta Joint stiffness values [7 joints]
         */
        void setJointImpedance(const std::array<double, 7>& K_theta);
        
        /**
         * Set Cartesian impedance parameters
         * @param K_x Cartesian stiffness values [x, y, z, rx, ry, rz]
         */
        void setCartesianImpedance(const std::array<double, 6>& K_x);
        
        /**
         * Configure collision detection behavior
         * @param lower_torque_thresholds_acceleration Lower torque thresholds for acceleration
         * @param upper_torque_thresholds_acceleration Upper torque thresholds for acceleration
         * @param lower_torque_thresholds_nominal Lower torque thresholds for nominal operation
         * @param upper_torque_thresholds_nominal Upper torque thresholds for nominal operation
         * @param lower_force_thresholds_acceleration Lower force thresholds for acceleration
         * @param upper_force_thresholds_acceleration Upper force thresholds for acceleration
         * @param lower_force_thresholds_nominal Lower force thresholds for nominal operation
         * @param upper_force_thresholds_nominal Upper force thresholds for nominal operation
         */
        void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                            const std::array<double, 7>& upper_torque_thresholds_acceleration,
                            const std::array<double, 7>& lower_torque_thresholds_nominal,
                            const std::array<double, 7>& upper_torque_thresholds_nominal,
                            const std::array<double, 6>& lower_force_thresholds_acceleration,
                            const std::array<double, 6>& upper_force_thresholds_acceleration,
                            const std::array<double, 6>& lower_force_thresholds_nominal,
                            const std::array<double, 6>& upper_force_thresholds_nominal);
        
        /**
         * Set load parameters for the robot end-effector
         * @param load_mass Mass of the load in kg
         * @param F_x_Cload Center of mass of the load relative to flange frame
         * @param load_inertia Inertia matrix of the load
         */
        void setLoad(double load_mass,
               const std::array<double, 3>& F_x_Cload,
               const std::array<double, 9>& load_inertia);

        /**
         * Load robot kinematic and dynamic model
         * @return Franka robot model
         */
        franka::Model loadModel();

    private:
        // Pimpl idiom: Private implementation pointer
        struct Implementation;
        std::unique_ptr<Implementation> impl_;
};

#endif // _FRANKA_HEADER_