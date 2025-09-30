#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <force_control_impedance/impedance_controller.h>
#include <force_control_impedance/config_deserialize_impedance.h>
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
    ImpedanceController::ImpedanceControllerConfig impedance_config;

    // open file
    const std::string CONFIG_PATH =
        "/home/robotlab/ACP/hardware_interfaces/applications/force_control_demo/config/franka_force_demo.yaml";

    // load config
    YAML::Node config{};
    try {
        // open and parse config file
        config = YAML::LoadFile(CONFIG_PATH);
        // deserialize robot and controller config
        robot_config.deserialize(config["franka"]);
        // deserialize controller config
        deserialize(config["impedance_controller"], impedance_config);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load the config file: " << e.what() << std::endl;
        return -1;
    }

    FRANKA robot(robot_config);
    ImpedanceController controller;
    RUT::Timer timer;
    RUT::TimePoint time0 = timer.tic();
    RUT::Vector7d pose, pose_ref, torque_cmd = RUT::Vector7d::Zero();
    RUT::Vector6d wrench, wrench0, wrench_WTr;

    //set impedance to robot
    robot.setJointImpedance(robot_config.setJointImpedance);
    robot.setCartesianImpedance(robot_config.setCartesianImpedance);
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

    // call robot model
    franka::Model model(robot.loadModel());

    //start motion with the configured modes
    robot.startMotion(
        research_interface::robot::Move::ControllerMode::kExternalController,
        research_interface::robot::Move::MotionGeneratorMode::kJointVelocity,
        {robot_config.deviation[0], robot_config.deviation[1], robot_config.deviation[2]},
        {robot_config.deviation[0], robot_config.deviation[1], robot_config.deviation[2]}
    );  

    // get initial pose, velocity and jacobian
    robot.getCartesian(pose);
    franka::RobotState state = robot.getRobotState();
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    //set jacobian and velocity to zero at the begining
    controller.getJacobian(jacobian);
    controller.getVelocity(dq);

    // Average the initial wrench over 2 seconds (200 samples at 1 kHz)
    wrench0.setZero();
    int avg_samples = 200;
    for (int i = 0; i < avg_samples; ++i) {
        RUT::Vector6d w;
        robot.getCurrentWrenchTool(w);
        wrench0 += w;
        usleep(10000); // 10 ms per sample (100 Hz)
    }
    wrench0 /= avg_samples;
    std::cout << "Averaged wrench0: " << wrench0.transpose() << std::endl;
    std::cout << "Starting control ..." << std::endl;

    // initialize controller
    controller.init(time0, impedance_config, pose);

    // transformation and number of force controlled axes
    RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
    int n_af = 6;
    controller.setForceControlledAxis(Tr, n_af);

    // desired reference pose and zero force reference
    pose_ref = pose;
    wrench_WTr.setZero();

    try
    {

        timer.set_loop_rate_hz(1000); // 1 kHz control loop
        timer.tic();

        while (true)
        {
            //print target pose with timestamp
            double dt = timer.toc_ms();
            timer.sleep_till_next();

            // print the robot pose
            //printf("Current pose before update: %f %f %f %f %f %f %f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);
            
            //we cant use getCartesian and getWrenchTool cause we are pooling the robot twice otherwise per loop
            // instea we use getCurrentWrenchTool in this case to just update wrench values with the getCurrentPose call
            robot.getCurrentPose(pose);
            robot.getCurrentWrenchTool(wrench);

            //get jacobian and velocity
            state = robot.getRobotState();
            jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

            // set jacobian and velocity in the controller (direct mapping without intermediate variables)
            controller.getJacobian(Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data()));
            controller.getVelocity(Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.dq.data()));

            // updates internal state with current pose and measured wrench
            controller.setRobotStatus(pose, wrench - wrench0);

            // Update robot reference
            controller.setRobotReference(pose_ref, wrench_WTr);

            // Compute the control output
            controller.step(torque_cmd);

            //printf("t = %f, target pose after update: %f %f %f %f %f %f %f\n", dt, pose_cmd[0], pose_cmd[1],
                //pose_cmd[2], pose_cmd[3], pose_cmd[4], pose_cmd[5], pose_cmd[6]);

            //during first iteration, send the 
            if (!robot.setTorques(torque_cmd)) {
                printf("setTorques failed\n");
                break;
            }

            //print torque command
            //printf("t = %f, torque_cmd: %f %f %f %f %f %f %f\n", dt, torque_cmd[0], torque_cmd[1],
                //torque_cmd[2], torque_cmd[3], torque_cmd[4], torque_cmd[5], torque_cmd[6]);

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
