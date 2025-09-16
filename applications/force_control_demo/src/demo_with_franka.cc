#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <unistd.h>
#include <franka/franka.h>
#include <yaml-cpp/yaml.h>

// function to deserialize eigen matrix from yaml
Eigen::MatrixXd deserialize_matrix(const YAML::Node& node) {
  int nr = node.size();
  int nc = node[0].size();
  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(nr, nc);
  for (int r = 0; r < nr; ++r) {
    for (int c = 0; c < nc; ++c) {
      mat(r, c) = node[r][c].as<double>();
    }
  }
  return mat;
}

// load eigen 
template <typename T>
T deserialize_vector(const YAML::Node& node) {
  std::vector<double> q = node.as<std::vector<double>>();
  return Eigen::Map<T, Eigen::Unaligned>(q.data(), q.size());
}

int main() {
    FRANKA::FRANKAConfig robot_config;
    AdmittanceController::AdmittanceControllerConfig admittance_config;

    // open file
    const std::string CONFIG_PATH =
        "/home/mikel/ACP/hardware_interfaces/applications/force_control_demo/config/franka_force_demo.yaml";

    // load config
    YAML::Node config{};
    try {
        // open and parse config file
        config = YAML::LoadFile(CONFIG_PATH);
        // deserialize robot and controller config
        robot_config.deserialize(config["franka"]);
        // deserialize controller config
        deserialize(config["admittance_controller"], admittance_config);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what() << std::endl;
        return -1;
    }

    FRANKA robot(robot_config);
    AdmittanceController controller;
    RUT::Timer timer;
    RUT::TimePoint time0 = timer.tic();
    RUT::Vector7d pose, pose_ref, pose_cmd;
    RUT::Vector6d wrench, wrench0, wrench_WTr;

    //set impedance to robot
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    robot.setCollisionBehavior(
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    //set load to robot
    robot.setLoad(robot_config.tcp_mass, {0.0, 0.0, 0.1},
                        {0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0});

    // Start the motion with Joint Impedance control and Joint Position motion generator
    robot.startCartesianMotion(
        research_interface::robot::Move::ControllerMode::kCartesianImpedance,
        research_interface::robot::Move::MotionGeneratorMode::kCartesianPosition
    );

    // get initial pose
    robot.getCartesian(pose);
    robot.getCartesian(pose);
    robot.getWrenchTool(wrench0);

    // get average wrench
    std::cout << "wrench0: " << wrench0.transpose() << std::endl;
    std::cout << "Starting in 2 seconds ..." << std::endl;
    wrench0.setZero();
    wrench_WTr.setZero();
    // desired reference pose and zero force reference
    pose_ref = pose;
    controller.setRobotReference(pose_ref, wrench_WTr);
    controller.step(pose_ref); // prime once
    sleep(2.0);

    // initialize controller
    controller.init(time0, admittance_config, pose);

    // transformation and number of force controlled axes
    // Start with regular admittance: all 6 axes are force-controlled
    RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
    int n_af = 1;
    controller.setForceControlledAxis(Tr, n_af);

    try
    {

        timer.tic();

        // Timer for control loop timing
        RUT::Timer loop_timer;

        while (true)
        {

            double dt = timer.toc_ms();

            // Update robot status
            robot.getCartesian(pose);
            robot.getWrenchTool(wrench);

            // updates internal state with current pose and measured wrench
            controller.setRobotStatus(pose, wrench - wrench0);

            // Update robot reference
            controller.setRobotReference(pose_ref, wrench_WTr);

            // Compute the control output
            controller.step(pose_cmd);

            if (!robot.setCartesian(pose_cmd)) {
            printf("setCartesian failed\n");
            break;
            }

            
            //printf("t = %f, wrench: %f %f %f %f %f %f\n", dt, wrench[0], wrench[1],
                //wrench[2], wrench[3], wrench[4], wrench[5]);

            if (dt > 50000) {
                // End motion
                robot.finishCurrentMotion();
                break;
            }

        }

    } catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
