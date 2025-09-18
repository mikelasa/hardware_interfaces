
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
    config.kDeltaT = 1e-4; // Time step for filtering
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

    // Quintic trajectory generator
    auto quintic_trajectory = [](const RUT::Vector3d& p0, const RUT::Vector3d& pf, double t, double T) {
        double tau = std::min(std::max(t / T, 0.0), 1.0);
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;
        double s = 10 * tau3 - 15 * tau4 + 6 * tau5;
        return p0 + s * (pf - p0);
    };

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

    // Timer for control loop timing
    RUT::Timer timer;

    RUT::Vector3d target_position(0.430179, 0.0, 0.520758); // desired target
    // trajectory time from prompt
    double trajectory_duration = 10.0; // seconds
    std::cout << "choose trajectory duration in seconds (e.g., 10.0): " << std::endl;
    std::cin >> trajectory_duration;

    // Precompute trajectory points, time steps 1e-3 (1kHz)
    int traj_steps = static_cast<int>(trajectory_duration * 1000);
    std::vector<RUT::Vector3d> trajectory(traj_steps);
    for (int i = 0; i < traj_steps; ++i) {
        double t = static_cast<double>(i) / 1000.0;
        trajectory[i] = quintic_trajectory(pose_ref.head<3>(), target_position, t, trajectory_duration);
    }
    
    try
    {
        // Start the timer for the control loop
        timer.tic();
        int step = 0;
        while (true) {

            // Get elapsed time in milliseconds
            double dt = timer.toc_ms();

            // Use precomputed trajectory
            if (step < traj_steps) {
                pose_ref[0] = trajectory[step][0];
                pose_ref[1] = trajectory[step][1];
                pose_ref[2] = trajectory[step][2];
            }

            // Send the new pose to the robot in real time
            if (!franka_robot.setCartesian(pose_ref)) {
                printf("setCartesian failed\n");

                break;
            }

            //get wrench at the tool OJO!!! si se hace readOnce dentro peta
            //franka_robot.getWrenchTool(wrench);
            //std::cout << "Current wrench at tool: " << wrench.transpose() << std::endl;
            //franka_robot.getWrenchBaseOnTool(wrench);
            //std::cout << "Current wrench at base (from tool): " << wrench.transpose() << std::endl;

            // Stop when trajectory is complete
            step++;

            if (step >= traj_steps)
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