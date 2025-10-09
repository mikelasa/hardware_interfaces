/**
 * FRANKA ROBOT TEST: Cartesian Motion Control Test
 * 
 * This test demonstrates basic Cartesian motion control with the Franka robot.
 * The robot performs a sinusoidal motion in the X-axis while maintaining
 * its initial position and orientation.
 * 
 * Test sequence:
 * 1. Initialize robot with configuration parameters
 * 2. Get initial pose and hold position for 30 cycles
 * 3. Execute sinusoidal motion for 10 seconds
 * 4. Finish motion and exit
 */

#include "franka/franka.h"
#include "robot_impl.h"
#include "franka/model.h"
#include <chrono>
#include <fstream> 
#include <RobotUtilities/timer_linux.h>

using namespace std::chrono;

/**
 * Load delta values from CSV file (currently unused in this test)
 * @param filename Path to CSV file containing delta values
 * @return Vector of double values loaded from file
 */
std::vector<double> loadDeltasFromCSV(const std::string& filename) {
    std::vector<double> deltas;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open CSV file: " << filename << std::endl;
        return deltas;
    }

    std::string line;
    while (std::getline(file, line)) {
        try {
            if (!line.empty()) {
                deltas.push_back(std::stod(line));  // one value per row
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing line: " << line << " -> " << e.what() << std::endl;
        }
    }

    return deltas;
}

int main() {
    
    // ========== Robot Configuration Setup ==========
    
    FRANKA::FRANKAConfig config;
    
    // Network and basic parameters
    config.robot_ip = "172.17.6.164";                      // Robot IP address
    config.log_size = 50;                                  // Log buffer size
    
    // Tool parameters (no tool attached)
    config.tcp_mass = 0.0;                                 // Tool mass in kg
    config.fx_c_load = {0.0, 0.0, 0.0};                   // Tool center of mass
    config.tcp_inertia = {0.0, 0.0, 0.0,                  // Tool inertia matrix
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0};
    
    // Motion parameters
    config.deviation = {10.0, 3.12, 2 * M_PI};            // Maximum path deviation
    config.kDeltaT = 1e-3;                                 // Time step for filtering
    config.CutoffFrequency = 100;                          // Cutoff frequency for low-pass filter
    
    // Control modes
    config.realtime_config = "ignore";                     // Realtime enforcement mode
    config.controller_mode = "cartesian_impedance";        // Controller type
    config.motion_generator_mode = "cartesian_position";   // Motion generator type
    
    // Impedance parameters
    config.setJointImpedance = {3000, 3000, 3000, 2500, 2500, 2000, 2000};     // Joint stiffness
    config.setCartesianImpedance = {1000, 1000, 1000, 200, 200, 200};          // Cartesian stiffness
    
    // Motion limits (reduced for safety)
    config.kMaxTranslationalVelocity = 0.25;              // Max translation velocity (m/s)
    config.kMaxTranslationalAcceleration = 1.0;           // Max translation acceleration (m/s²)
    config.kMaxTranslationalJerk = 500;                   // Max translation jerk (m/s³)
    config.kMaxRotationalVelocity = 0.25;                 // Max rotation velocity (rad/s)
    config.kMaxRotationalAcceleration = 1.0;              // Max rotation acceleration (rad/s²)
    config.kMaxRotationalJerk = 500;                      // Max rotation jerk (rad/s³)

    // ========== Safety Configuration ==========
    
    // Set safety and operation modes for the robot
    config.robot_interface_config.zone_safety_mode =
        RobotSafetyMode::SAFETY_MODE_TRUNCATE;             // Truncate commands outside safe zone
    config.robot_interface_config.incre_safety_mode =
        RobotSafetyMode::SAFETY_MODE_STOP;                 // Stop on excessive increments
    config.robot_interface_config.operation_mode =
        RobotOperationMode::OPERATION_MODE_CARTESIAN;      // Cartesian operation mode
    config.robot_interface_config.max_incre_m = 0.002;    // Max position increment (2mm per cycle)
    config.robot_interface_config.max_incre_rad = 0.00628;// Max orientation increment (0.36° per cycle)
    config.robot_interface_config.safe_zone = {0.3, 0.65, -0.3, 0.4, 0.1, 0.4}; // Safe workspace bounds
    
    // ========== Robot Initialization ==========
    
    // Instantiate Franka robot interface
    FRANKA franka_robot;
    RUT::Timer timer;
    
    // Initialize the robot with the configuration
    franka_robot.init(timer.tic(), config);
    
    // ========== Initial State Setup ==========
    
    // Get the initial Cartesian pose of the robot
    RUT::Vector7d pose0;
    franka_robot.getCartesian(pose0);
    RUT::Vector7d pose_ref = pose0;
    
    // Print initial pose for reference
    std::cout << "Initial Cartesian pose: " << pose_ref.transpose() << std::endl;

    // Get wrench at the tool (declared but not used in this test)
    RUT::Vector6d wrench;

    // ========== Position Stabilization ==========
    
    // Send pose0 directly, do not modify or reconstruct
    // This ensures the robot holds its initial position before starting motion
    for (int i = 0; i < 30; ++i) {
        franka_robot.setCartesian(pose0);
    }

    // ========== Control Loop Setup ==========
    
    timer.set_loop_rate_hz(1000);                         // 1 kHz control loop frequency

    // Timer variables for motion generation
    double time = 0.0;                                     // Accumulated time counter
    franka::Duration previous_time = franka_robot.getElapsedTime();
    franka::Duration period;

    try {
        // ========== Main Control Loop ==========
        
        // Start the timer for the control loop
        timer.tic();
        
        while (true) {

            // ========== Time Management ==========
            
            // Using RUT::Timer for external timing reference
            double timer_time = timer.toc_ms() / 1000.0;

            // Alternative pose reading (commented out for this test)
            //franka_robot.getCurrentPose(pose0);

            // Sleep until next control cycle
            timer.sleep_till_next();
            
            // Update internal time tracking using robot's elapsed time
            period = franka_robot.getElapsedTime() - previous_time;
            previous_time = franka_robot.getElapsedTime();
            time += period.toSec();

            // ========== Status Reporting ==========
            
            // Print timing information every second
            if (time - int(time) < 0.001) {
                std::cout << "Timer time: " << timer_time << " s, "
                    << "time: " << time << " s," << std::endl;
            }
                
            // ========== Motion Generation ==========
            
            constexpr double kRadius = 0.1;                // Motion amplitude (100mm)
            
            // Generate sinusoidal motion profile
            // Creates a smooth acceleration/deceleration using cosine envelope
            double angle = M_PI / 4 * (1 - std::cos(M_PI / 5 * time));
            double delta_x = kRadius * std::sin(angle);
            
            // Apply motion only to X-axis, keeping other coordinates unchanged
            pose_ref = pose0;
            pose_ref[0] += delta_x;

            // ========== Command Execution ==========
            
            // Send the new pose to the robot in real time
            if (!franka_robot.setCartesian(pose_ref)) {
                printf("setCartesian failed\n");
                break;
            }
            
            // ========== Test Completion Check ==========
            
            if (time >= 10) {
                std::cout << "Finished Cartesian motion test." << std::endl;
                // End motion gracefully
                franka_robot.finishCurrentMotion();
                break;
            }
        }
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    return 0;
}