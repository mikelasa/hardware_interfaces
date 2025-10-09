/**
 * FRANKA FORCE CONTROL DEMO: Impedance Controller Integration Test
 * 
 * This demo demonstrates the integration of the Franka robot with the impedance
 * controller for force control applications. The robot maintains a fixed pose
 * while the impedance controller regulates forces and torques.
 * 
 * Demo sequence:
 * 1. Load configuration from YAML file
 * 2. Initialize robot and impedance controller
 * 3. Set up force-controlled axes (all 6 DOF)
 * 4. Run control loop with torque commands for 50 seconds
 * 5. Finish motion and exit
 */

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <force_control_impedance/impedance_controller.h>
#include <force_control_impedance/config_deserialize_impedance.h>
#include <unistd.h>
#include <franka/franka.h>
#include <yaml-cpp/yaml.h>

// ========== Helper Functions ==========

/**
 * Deserialize Eigen matrix from YAML node
 * @param node YAML node containing 2D array data
 * @return Eigen::MatrixXd matrix populated with YAML data
 */
Eigen::MatrixXd deserialize_matrix(const YAML::Node& node) {
    int nr = node.size();                                   // Number of rows
    int nc = node[0].size();                                // Number of columns
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(nr, nc);
    for (int r = 0; r < nr; ++r) {
        for (int c = 0; c < nc; ++c) {
            mat(r, c) = node[r][c].as<double>();
        }
    }
    return mat;
}

/**
 * Deserialize Eigen vector from YAML node
 * @param node YAML node containing 1D array data
 * @return Eigen vector of specified type T
 */
template <typename T>
T deserialize_vector(const YAML::Node& node) {
    std::vector<double> q = node.as<std::vector<double>>();
    return Eigen::Map<T, Eigen::Unaligned>(q.data(), q.size());
}

int main() {
    
    // ========== Configuration Setup ==========
    
    FRANKA::FRANKAConfig robot_config;
    ImpedanceController::ImpedanceControllerConfig impedance_config;

    // Configuration file path
    const std::string CONFIG_PATH =
        "/home/robotlab/ACP/hardware_interfaces/applications/force_control_demo/config/franka_force_demo.yaml";

    // ========== Configuration Loading ==========
    
    YAML::Node config{};
    try {
        // Open and parse configuration file
        config = YAML::LoadFile(CONFIG_PATH);
        
        // Deserialize robot configuration
        robot_config.deserialize(config["franka"]);
        
        // Deserialize impedance controller configuration
        deserialize(config["impedance_controller"], impedance_config);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what() << std::endl;
        return -1;
    }

    // ========== Object Initialization ==========
    
    FRANKA robot;
    ImpedanceController controller;
    RUT::Timer timer;
    RUT::TimePoint time0 = timer.tic();
    
    // State variables
    RUT::Vector7d pose, pose_ref, torque_cmd = RUT::Vector7d::Zero();
    RUT::Vector6d wrench, wrench0, wrench_WTr;

    // ========== Robot Initialization ==========
    
    // Initialize the robot with the configuration
    robot.init(time0, robot_config);

    // ========== Initial State Acquisition ==========
    
    // Get initial pose, velocity and jacobian
    robot.getCartesian(pose);
    franka::Model model = franka::Model(robot.loadModel());
    franka::RobotState state = robot.getRobotState();
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    // Set jacobian and velocity to zero at the beginning
    controller.getJacobian(jacobian);
    controller.getRobotState(state);

    std::cout << "Starting control loop..." << std::endl;

    // ========== Controller Initialization ==========
    
    // Initialize impedance controller
    controller.init(time0, impedance_config, pose);

    // ========== Force Control Configuration ==========
    
    // Transformation matrix and number of force controlled axes
    RUT::Matrix6d Tr = RUT::Matrix6d::Identity();           // Identity transformation matrix
    int n_af = 6;                                           // All 6 DOF force-controlled
    controller.setForceControlledAxis(Tr, n_af);

    // ========== Reference Setup ==========
    
    // Desired reference pose and zero force reference
    pose_ref = pose;                                        // Hold initial pose
    wrench_WTr.setZero();                                   // Zero force reference

    // Fixed pose for testing (same as initial pose)
    RUT::Vector7d test_pose = pose_ref;

    try {
        
        // ========== Control Loop Setup ==========
        
        timer.set_loop_rate_hz(1000);                       // 1 kHz control loop frequency
        timer.tic();

        while (true) {
            
            // ========== Timing Management ==========
            
            // Print target pose with timestamp
            double dt = timer.toc_ms();                      // Elapsed time in milliseconds
            timer.sleep_till_next();                         // Sleep until next control cycle
            
            // ========== State Updates ==========
            
            /* UPDATE VALUES */
            // We can't use getCartesian and getWrenchTool because we would be polling the robot twice per loop
            // Instead we use getCurrentWrenchTool to just update wrench values with the getCurrentPose call
            robot.getCurrentPose(pose);
            robot.getCurrentWrenchTool(wrench);

            // Get jacobian and velocity from current robot state
            state = robot.getRobotState();
            jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

            // ========== Controller Updates ==========
            
            // Set jacobian and velocity in the controller (direct mapping without intermediate variables)
            controller.getJacobian(Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data()));
            controller.getRobotState(state);

            // Updates internal state with current pose and measured wrench
            controller.setRobotStatus(pose, wrench);

            // Update robot reference (maintain fixed pose with zero force)
            controller.setRobotReference(test_pose, wrench_WTr);

            // ========== Control Computation ==========
            
            // Compute the control output (torque commands)
            controller.step(torque_cmd);

            // ========== Command Execution ==========
            
            // During first iteration, send the computed torques
            if (!robot.setTorques(torque_cmd)) {
                printf("setTorques failed\n");
                break;
            }

            // ========== Debug Output ==========
            
            // Print torque command (commented out for performance)
            //printf("t = %f, torque_cmd: %f %f %f %f %f %f %f\n", dt, torque_cmd[0], torque_cmd[1],
                //torque_cmd[2], torque_cmd[3], torque_cmd[4], torque_cmd[5], torque_cmd[6]);

            // ========== Demo Completion Check ==========
            
            if (dt > 50000) {                                // Run for 50 seconds
                // End motion gracefully
                robot.finishCurrentMotion();
                std::cout << "FIN" << std::endl;
                break;
            }
        }

    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    return 0;
}