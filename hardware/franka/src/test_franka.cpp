
#include "franka/franka.h"
#include "robot_impl.h"
#include "franka/model.h"
#include <chrono>
#include <fstream> 
#include <RobotUtilities/timer_linux.h>


using namespace std::chrono;


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
    FRANKA::FRANKAConfig config;
    config.robot_ip = "172.17.6.164";
    config.log_size = 50;
    config.tcp_mass = 0.0;
    config.fx_c_load = {0.0, 0.0, 0.0}; // default origin
    config.tcp_inertia = {0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0}; // default inertia
    config.deviation = {10.0, 3.12, 2 * M_PI}; // default deviation
    config.kDeltaT = 1e-3; // Time step for filtering
    config.CutoffFrequency = 100; // Cutoff frequency for low-pass filter
    config.realtime_config = "ignore";
    config.setJointImpedance = {3000, 3000, 3000, 2500, 2500, 2000, 2000};
    config.setCartesianImpedance = {1000, 1000, 1000, 200, 200, 200};
    config.kMaxTranslationalVelocity = 0.25;
    config.kMaxTranslationalAcceleration = 1.0;
    config.kMaxTranslationalJerk = 500;
    config.kMaxRotationalVelocity = 0.25;
    config.kMaxRotationalAcceleration = 1.0;
    config.kMaxRotationalJerk = 500;

    // set safety and operation modes for the robot
    config.robot_interface_config.zone_safety_mode =
        RobotSafetyMode::SAFETY_MODE_TRUNCATE;
    config.robot_interface_config.incre_safety_mode =
        RobotSafetyMode::SAFETY_MODE_STOP;
    config.robot_interface_config.operation_mode =
        RobotOperationMode::OPERATION_MODE_CARTESIAN;
    config.robot_interface_config.max_incre_m = 0.002;      // 1 m per second
    config.robot_interface_config.max_incre_rad = 0.00628;  // 3.14 per second
    config.robot_interface_config.safe_zone = {0.3, 0.65, -0.3, 0.4, 0.1, 0.4};
    
    // Instantiate Franka robot interface
    FRANKA franka_robot;
    RUT::Timer timer;
    // Initialize the robot with the configuration
    franka_robot.init(timer.tic(), config);

    //set impedance to robot
    franka_robot.setJointImpedance(config.setJointImpedance);
    franka_robot.setCartesianImpedance(config.setCartesianImpedance);
    franka_robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        
    //set load to robot
    franka_robot.setLoad(config.tcp_mass, config.fx_c_load, config.tcp_inertia);

    //start motion with the configured modes
    franka_robot.startMotion(
        research_interface::robot::Move::ControllerMode::kCartesianImpedance,
        research_interface::robot::Move::MotionGeneratorMode::kCartesianPosition,
        {config.deviation[0], config.deviation[1], config.deviation[2]},
        {config.deviation[0], config.deviation[1], config.deviation[2]}
    );
    

    // Get the initial Cartesian pose of the robot
    RUT::Vector7d pose0;
    franka_robot.getCartesian(pose0);
    RUT::Vector7d pose_ref = pose0;
    //print initial pose
    std::cout << "Initial Cartesian pose: " << pose_ref.transpose() << std::endl;

    //get wrench at the tool
    RUT::Vector6d wrench;

    // Send pose0 directly, do not modify or reconstruct
    for (int i = 0; i < 30; ++i) {
        franka_robot.setCartesian(pose0);
    }

    timer.set_loop_rate_hz(1000); // 1 kHz control loop

    // to test timers
    double time = 0.0;
    franka::Duration previous_time = franka_robot.getElapsedTime();
    franka::Duration period;

    try
    {
        // Start the timer for the control loop
        timer.tic();
        
        while (true) {

            // Using RUT::Timer
            double timer_time = timer.toc_ms() / 1000.0;

            //franka_robot.getCurrentPose(pose0);

            // Get elapsed time in milliseconds
            timer.sleep_till_next();
            period = franka_robot.getElapsedTime() - previous_time;
            previous_time = franka_robot.getElapsedTime();
            time += period.toSec();

            // print every second
            if  (time - int(time) < 0.001 )
                {
                    std::cout << "Timer time: " << timer_time << " s, "
                        << "time: " << time << " s," << std::endl;
                }
                
            constexpr double kRadius = 0.1; // Reduced amplitude for safer delta_x
            double angle = M_PI / 4 * (1 - std::cos(M_PI / 5 * time));
            double delta_x = kRadius * std::sin(angle);
            pose_ref = pose0;
            //pose_ref[0] += delta_x;

            // Send the new pose to the robot in real time
            if (!franka_robot.setCartesian(pose0)) {
                printf("setCartesian failed\n");

                break;
            }
            
            if (time >= 10)
            {
                std::cout << "Finished Cartesian motion test." << std::endl;
                // End motion
                franka_robot.finishCurrentMotion();
                break;
            }

        }
    }

    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}