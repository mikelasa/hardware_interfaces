#include "franka/franka.h"
#include "robot_impl.h"
#include "franka/model.h"
#include <chrono>

using namespace std::chrono;

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
    config.realtime_config = "ignore";
    config.setJointImpedance = {3000, 3000, 3000, 2500, 2500, 2000, 2000};
    config.setCartesianImpedance = {1000, 1000, 1000, 200, 200, 200};

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
    FRANKA franka_robot(config);

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
        research_interface::robot::Move::ControllerMode::kJointImpedance,
        research_interface::robot::Move::MotionGeneratorMode::kJointPosition,
        {config.deviation[0], config.deviation[1], config.deviation[2]},
        {config.deviation[0], config.deviation[1], config.deviation[2]}
    );

    //get joint positions
    RUT::VectorXd q;
    RUT::VectorXd pos_ref(7);
    franka_robot.getJoints(q);
    pos_ref = q;
    std::cout << "Current joint positions: " << q.transpose() << std::endl;

    //get wrench at the tool
    RUT::Vector6d wrench;

    // Timer for control loop timing
    RUT::Timer timer;

    try
    {
        // Start the timer for the control loop
        timer.tic();
        
        while (true) {

            // Get elapsed time in milliseconds
            double dt = timer.toc_ms();
            
            // compute a small delta for joint 5 motion
            double delta = 0.0001;

            // Define target position for joints
            pos_ref[4] += delta;
            
            if (!franka_robot.setJoints(pos_ref)) {
                printf("setJoints failed\n");

                break;
            }
            
            //get wrench at the tool (TEST)
            //franka_robot.getWrenchTool(wrench);

            //print target position
            std::cout << "Target joint positions: " << pos_ref.transpose() << std::endl;

            if (dt >= 5000)  // run for 5 seconds
            {
                // End motion
                std::cout << "Stopping motion after 5 seconds." << std::endl;
                franka_robot.finishCurrentMotion();
                std::cout << "Finished Cartesian motion test." << std::endl;
                break;
            }

        }

    }catch (const std::exception& e) {
                std::cerr << "Error:  " << e.what() << std::endl;
                return -1;
            }

    std::cout << "finish" << std::endl;
    return 0;
}