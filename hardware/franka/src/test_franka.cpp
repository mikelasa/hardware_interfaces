
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
    config.tcp_inertia = 0.01;
    config.deviation = {10.0, 3.12, 2 * M_PI}; // default deviation
    config.kDeltaT = 1e-6; // Time step for filtering
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
    franka_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    franka_robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    franka_robot.setCollisionBehavior(
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    //set load to robot
    franka_robot.setLoad(config.tcp_mass, {0.0, 0.0, 0.1},
                        {0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0});

    // Start the motion with Joint Impedance control and Joint Position motion generator
    franka_robot.startCartesianMotion(
        research_interface::robot::Move::ControllerMode::kCartesianImpedance,
        research_interface::robot::Move::MotionGeneratorMode::kCartesianPosition
    );

    // Get the initial Cartesian pose of the robot
    RUT::Vector7d pose0;
    franka_robot.getCartesian(pose0);
    RUT::Vector7d pose_ref = pose0;

    //get wrench at the tool
    RUT::Vector6d wrench;

    //print initial pose
    std::cout << "Initial Cartesian pose: " << pose_ref.transpose() << std::endl;

    // Timer for control loop timing
    RUT::Timer timer;

    RUT::Vector3d target_position(0.430179, 0.0, 0.520758); // desired target
    double trajectory_duration = 20; // seconds

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

            //get wrench at the tool (TEST)
            franka_robot.getWrenchTool(wrench);
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