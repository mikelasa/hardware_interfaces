#include "franka/franka.h"
#include "robot_impl.h"
#include "lowpass_filter.h"
#include "rate_limiting.h"
#include "franka/model.h"
#include <chrono>
#include <fstream> 
using namespace std::chrono;

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}


int main() {
    FRANKA::FRANKAConfig config;
    config.robot_ip = "172.17.6.164";
    config.log_size = 50;
    config.tcp_mass = 0.5;
    config.tcp_inertia = {0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0}; // default inertia
    config.deviation = {10.0, 3.12, 2 * M_PI}; // Example deviation values

    research_interface::robot::Move::Deviation deviation{
        config.deviation[0],
        config.deviation[1],
        config.deviation[2]
    };

    // Instantiate Franka robot interface
    FRANKA franka_robot;
    RUT::Timer timer;
    // Initialize the robot with the configuration
    franka_robot.init(timer.tic(), config);

    research_interface::robot::MotionGeneratorCommand motion_command{};
    research_interface::robot::ControllerCommand control_command{};
    
    RUT::Vector6d wrench;

    double kDeltaT = 1e-3;
    
    // pick a test case to run with user input from prompt
    int test_case = 0;
    std::cout << "Select a test case to run (1-5):" << std::endl;
    std::cout << "1: Get Robot State" << std::endl;
    std::cout << "2: Get Robot State through update" << std::endl;
    std::cout << "3: Start and immediately finish motion" << std::endl;
    std::cout << "4: Start motion, move in torques and stop" << std::endl;
    std::cout << "5: Start motion, move in position and stop" << std::endl;
    std::cout << "6: Start motion, move in cartesian and stop" << std::endl;
    std::cout << "Enter test case number: ";
    std::cin >> test_case;
    

    switch (test_case) {
        case 1:
            // Test 1: Get Robot State
            //instancia la clase FRANKA con la configuracion
            try {

                //get states readOnce
                franka::RobotState state = franka_robot.readOnce();
                std::cout << "Robot state read successfully." << std::endl;
                std::cout << "state" << state << std::endl;

            } catch (const std::exception& e) {
                std::cerr << "Error al crear la instancia de FRANKA: " << e.what() << std::endl;
                return -1;
            }
            break;
        case 2:
            // Test 2: get robot state trough update
            try {
                //get states readOnce
                franka::RobotState state = franka_robot.update(nullptr, nullptr);
                std::cout << "Robot state updated successfully." << std::endl;
                std::cout << "state" << state << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Error al actualizar el estado del robot: " << e.what() << std::endl;
                return -1;
            }
            break;
        case 3: {
            std::cout << "[Test 3] Starting and immediately finishing motion session." << std::endl;

            try {
                uint32_t motion_id = franka_robot.startMotion(
                    research_interface::robot::Move::ControllerMode::kExternalController,
                    research_interface::robot::Move::MotionGeneratorMode::kJointPosition,
                    deviation, deviation
                );

                RUT::Timer timer;

                while ( timer.toc_ms() < 20000)
                {
                    franka_robot.getWrenchTool(wrench);
                    std::cout << "Current wrench at tool: " << wrench.transpose() << std::endl; 
                }
                
                franka_robot.finishMotion(motion_id, nullptr, nullptr);
                std::cout << "[Test 3] Motion session finished successfully." << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[Test 3] Failed to start/finish motion: " << e.what() << std::endl;
                return -1;
            }
            break;
        }
        case 4: {
            std::cout << "[Test 4] Start motion, move in torques and stop." << std::endl;
            try {
                uint32_t motion_id = franka_robot.startMotion(
                    research_interface::robot::Move::ControllerMode::kExternalController,
                    research_interface::robot::Move::MotionGeneratorMode::kJointVelocity,
                    deviation, deviation
                );

                // Set zero joint velocities (required if using kJointVelocity)
                motion_command.dq_c.fill(0.0);
                motion_command.motion_generation_finished = false;

                //read the current state of the robot
                franka::RobotState robot_state = franka_robot.readOnce();
                std::cout << "[Test 4] Robot state before motion: " << robot_state.q << std::endl;

                for (int i = 0; i < 1000; ++i) {
                    // añadir torque al eje 4 del robot
                    control_command.tau_J_d.fill(0.0);
                    control_command.tau_J_d[4] = -1; // Apply torque to joint 4

                    franka_robot.update(&motion_command, &control_command);
                    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                // Finish the motion session
                motion_command.motion_generation_finished = true;
                franka_robot.finishMotion(motion_id, &motion_command, &control_command);
                std::cout << "[Test 4] Motion session finished successfully." << std::endl;

                //read the current state of the robot
                robot_state = franka_robot.readOnce();
                std::cout << "[Test 4] Robot state after motion: " << robot_state.q << std::endl;

            } catch (const std::exception& e) {
                std::cerr << "[Test 4] Failed to start/finish motion: " << e.what() << std::endl;
                return -1;
            }
            break;
        }
        
        case 5: {
            std::cout << "[Test 5] Start motion, move in position and stop." << std::endl;

            try {

                //set impedance to robot
                franka_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
                franka_robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
                franka_robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});


                uint32_t motion_id = franka_robot.startMotion(
                    research_interface::robot::Move::ControllerMode::kJointImpedance,
                    research_interface::robot::Move::MotionGeneratorMode::kJointPosition,
                    deviation, deviation
                );

                // Read the initial robot state
                //franka::RobotState init_state = franka_robot.readOnce();
                franka::RobotState robot_state = franka_robot.update(nullptr, nullptr);
                franka_robot.throwOnMotionError(robot_state, motion_id);
                std::cout << "[Test 5] Robot state before motion: " << robot_state.q << std::endl;

                // Initial setup: capture initial state
                std::array<double, 7> target_position = robot_state.q;

                // franka duration for the control loop
                franka::Duration period;
                franka::Duration previous_time = robot_state.time;

                double time = 0.0;
                double kDeltaT = 1e-3;

                while(!motion_command.motion_generation_finished) {

                    // calculate the time step
                    period = robot_state.time - previous_time;
                    previous_time = robot_state.time;
                    time += period.toSec();

                    // Update target position based on time
                    double delta = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));

                    // here goes SpinMotion similar part
                    //move joint 5
                    motion_command.q_c = target_position;
                    motion_command.q_c[4] = motion_command.q_c[4] + delta;

                    std::cout << "motion_command.qc1: " << motion_command.q_c << std::endl;
                    std::cout << "robot_state.q_d: " << robot_state.q_d << std::endl;
                    // Apply low-pass filter from lowpass_filter.h to the target position
                    for (size_t i = 0; i < 7; ++i) {
                        motion_command.q_c[i] = franka::lowpassFilter(
                            kDeltaT,
                            motion_command.q_c[i],
                            robot_state.q_d[i],
                            franka::kDefaultCutoffFrequency
                        );
                    }
                    std::cout << "motion_command.qc2: " << motion_command.q_c << std::endl;
                    
                    //limit rate of the motion command
                    motion_command.q_c = franka::limitRate(
                        franka::kMaxJointVelocity,
                        franka::kMaxJointAcceleration,
                        franka::kMaxJointJerk,
                        motion_command.q_c,
                        robot_state.q_d,
                        robot_state.dq_d,
                        robot_state.ddq_d
                    );

                    std::cout << "motion_command.qc3: " << motion_command.q_c << std::endl;
                    // Update robot: send command and receive new state
                    robot_state = franka_robot.update(&motion_command, nullptr);
                    franka_robot.throwOnMotionError(robot_state, motion_id);
                    
                    // if time is 5
                    if (time >= 5.0) {
                        motion_command.motion_generation_finished = true;
                    }

                }

                std::cout << "[Test 5] Motion command finished." << std::endl;
                // Finish the motion session (deberia meter en el while con un flag cuando termina el movimiento)
                franka_robot.finishMotion(motion_id, &motion_command, nullptr);

                // Read final state
                franka::RobotState final_state = franka_robot.readOnce();
                std::cout << "[Test 5] Robot state after motion: " << final_state.q << std::endl;

            } catch (const std::exception& e) {
                std::cerr << "[Test 5] Failed to start/finish motion: " << e.what() << std::endl;
                return -1;
            }
            break;
        }

        case 6: {
            std::cout << "[Test 6] Start motion, torque control with smooth tracking." << std::endl;

            try {
                // Start motion with torque control and velocity motion generator (required)
                uint32_t motion_id = franka_robot.startMotion(
                    research_interface::robot::Move::ControllerMode::kExternalController,
                    research_interface::robot::Move::MotionGeneratorMode::kJointVelocity,
                    deviation, deviation
                );

                // Set initial velocity command (required by JointVelocity mode)
                motion_command.dq_c.fill(0.0);
                motion_command.motion_generation_finished = false;

                // Read initial state
                franka::RobotState robot_state = franka_robot.update(nullptr, nullptr);
                franka_robot.throwOnMotionError(robot_state, motion_id);
                std::array<double, 7> initial_position = robot_state.q;
                franka::Duration previous_time = robot_state.time;

                // Time tracking
                double time = 0.0;
                franka::Duration period;

                while (!motion_command.motion_generation_finished) {
                    // Compute time step
                    period = robot_state.time - previous_time;
                    previous_time = robot_state.time;
                    time += period.toSec();

                    // Motion generation: keep dq_c = 0 to stay still
                    motion_command.dq_c.fill(0.0);

                    // Control generation: apply smooth torque to joint 4
                    double torque = 1.5 * std::sin(2.0 * M_PI * 0.25 * time);  // 0.25 Hz sine wave
                    control_command.tau_J_d.fill(0.0);
                    control_command.tau_J_d[4] = torque;

                    // Apply low-pass filter to the torque command
                    for (size_t i = 0; i < 7; ++i) {
                        control_command.tau_J_d[i] = franka::lowpassFilter(
                            kDeltaT,
                            control_command.tau_J_d[i],
                            robot_state.tau_J_d[i],
                            franka::kDefaultCutoffFrequency
                        );
                    }

                    //rate limit the torque command
                    control_command.tau_J_d = franka::limitRate(
                        franka::kMaxTorqueRate,
                        control_command.tau_J_d,
                        robot_state.tau_J_d
                    );

                    // Send command, receive updated state
                    robot_state = franka_robot.update(&motion_command, &control_command);
                    franka_robot.throwOnMotionError(robot_state, motion_id);

                    // Stop after 5 seconds
                    if (time >= 5.0) {
                        motion_command.motion_generation_finished = true;
                    }

                }

                // Finish motion
                franka_robot.finishMotion(motion_id, &motion_command, &control_command);
                std::cout << "[Test 6] Motion session finished successfully." << std::endl;

                // Final state
                franka::RobotState final_state = franka_robot.readOnce();
                std::cout << "[Test 6] Robot state after motion: " << final_state.q << std::endl;

            } catch (const std::exception& e) {
                std::cerr << "[Test 6] Failed to start/finish motion: " << e.what() << std::endl;
                return -1;
            }

            break;
        }

        case 7: {
            std::cout << "[Test 7] Start Cartesian pose motion." << std::endl;

            try {

                // Optional: set impedance & collision if needed
                franka_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
                franka_robot.setCartesianImpedance({{1000, 1000, 1000, 200, 200, 200}});
                franka_robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

                //save directory
                std::string save_directory = "/home/robotlab/test_cases.csv";

                // A thread for reading states and the other for control.
                struct {
                    std::mutex mutex;
                    bool has_data;
                    std::array<double, 7> q_d;
                    std::array<double, 7> dq_d;
                    std::array<double, 7> ddq_d;
                    RUT::Vector6d O_dP_EE_c;
                    RUT::Vector6d O_ddP_EE_c;
                    double time;
                    double delta;
                } save_data{};
                std::atomic_bool running{true};

                //start print thread
                std::thread print_thread([&save_data, &running, save_directory]() {
                    std::ofstream file(save_directory);
                    if (!file) {
                        std::cerr << "Could not open file for writing: " << save_directory << std::endl;
                        return;
                    }

                        while (running) {
                            if (save_data.mutex.try_lock()) {
                                if (save_data.has_data) {
                                    for (const auto& val : save_data.q_d) file << val << ",";
                                    for (const auto& val : save_data.dq_d) file << val << ",";
                                    for (const auto& val : save_data.ddq_d) file << val << ",";
                                    for (const auto& val : save_data.O_dP_EE_c) file << val << ",";
                                    for (const auto& val : save_data.O_ddP_EE_c) file << val << ",";
                                    file << save_data.delta << ",";
                                    file << save_data.time << ",";
                                    file << "\n";
                                    save_data.has_data = false;
                                }
                                save_data.mutex.unlock();
                            }
                            
                        }

                        file.close();
                    });


                uint32_t motion_id = franka_robot.startMotion(
                    research_interface::robot::Move::ControllerMode::kCartesianImpedance,
                    research_interface::robot::Move::MotionGeneratorMode::kCartesianPosition,
                    deviation, deviation
                );

                // Read initial robot state
                franka::RobotState robot_state = franka_robot.readOnce();

                std::array<double, 16> initial_pose = robot_state.O_T_EE;
                franka::Duration previous_time = robot_state.time;
                franka::Duration period;
                double time = 0.0;

                //create timer from chrono
                while (!motion_command.motion_generation_finished) {
                    // Update time
                    period = robot_state.time - previous_time;
                    previous_time = robot_state.time;
                    time += period.toSec();

                    //print time
                    std::cout << "Time: " << time << " s" << std::endl;

                    constexpr double kRadius = 0.1;
                    double angle = M_PI / 4 * (1 - std::cos(M_PI / 3.0 * time));
                    double delta_x = kRadius * std::sin(angle);
                    double delta_z = kRadius * (std::cos(angle) - 1);

                    // Build new pose
                    motion_command.O_T_EE_c = initial_pose;
                    motion_command.O_T_EE_c[12] += delta_x;  // Move in X (element 12 of 4x4 matrix)
                    //motion_command.O_T_EE_c[14] += delta_z;  // Move in Z (element 14 of 4x4 matrix)

                    // Optionally apply low-pass filter (for smooth pose)
                    motion_command.O_T_EE_c = franka::cartesianLowpassFilter(
                        kDeltaT,
                        motion_command.O_T_EE_c,
                        robot_state.O_T_EE_c,
                        franka::kDefaultCutoffFrequency
                    );

                    // Limit rate of the motion command
                    motion_command.O_T_EE_c = franka::limitRate(
                        franka::kMaxTranslationalVelocity,
                        franka::kMaxTranslationalAcceleration,
                        franka::kMaxTranslationalJerk,
                        franka::kMaxRotationalVelocity,
                        franka::kMaxRotationalAcceleration,
                        franka::kMaxRotationalJerk,
                        motion_command.O_T_EE_c,
                        robot_state.O_T_EE_c,
                        robot_state.O_dP_EE_c,
                        robot_state.O_ddP_EE_c
                    );

                    //log data
                    if (save_data.mutex.try_lock()) {
                        save_data.q_d = robot_state.q_d;
                        save_data.dq_d = robot_state.dq_d;
                        save_data.ddq_d = robot_state.ddq_d;
                        save_data.O_dP_EE_c = Eigen::Map<const RUT::Vector6d>(robot_state.O_dP_EE_c.data(), 6);
                        save_data.O_ddP_EE_c = Eigen::Map<const RUT::Vector6d>(robot_state.O_ddP_EE_c.data(), 6);
                        save_data.delta = delta_x;
                        save_data.time = time;
                        save_data.has_data = true;
                        save_data.mutex.unlock();
                    }

                    // Update robot
                    robot_state = franka_robot.update(&motion_command, nullptr);
                    franka_robot.throwOnMotionError(robot_state, motion_id);

                    // Exit after 10s
                    if (time >= 10.0) {
                        motion_command.motion_generation_finished = true;
                    }

                } // <-- closes the while loop

                // End motion
                franka_robot.finishMotion(motion_id, &motion_command, nullptr);
                std::cout << "[Test 7] Cartesian motion finished." << std::endl;

                // Read final pose
                franka::RobotState final_state = franka_robot.readOnce();
                std::cout << "[Test 7] Final EE pose: ";
                for (int i = 0; i < 16; ++i) std::cout << final_state.O_T_EE[i] << " ";
                std::cout << std::endl;

                running = false;
                if (print_thread.joinable()) {
                    print_thread.join();
                }

            } catch (const std::exception& e) {
                std::cerr << "[Test 7] Cartesian motion error: " << e.what() << std::endl;
                return -1;
            }

            break;
        }

        case 8:
            std::cout << "[Test 8] Cartesian impedance control test." << std::endl;

            try {

                // Optional: set impedance & collision if needed
                franka_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
                franka_robot.setCartesianImpedance({{1000, 1000, 1000, 200, 200, 200}});
                franka_robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

                // Set initial velocity command (required by JointVelocity mode)
                motion_command.dq_c.fill(0.0);
                motion_command.motion_generation_finished = false;

                // Compliance parameters
                const double K_trans{100};
                const double K_rot{10};
                const double k_nullspace_trans{100};
                const double k_nullspace_rot{10};

                //create the impedance matrixes
                Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
                stiffness.setZero();
                stiffness.topLeftCorner(3, 3) << K_trans * Eigen::MatrixXd::Identity(3, 3);
                stiffness.bottomRightCorner(3, 3) << K_rot * Eigen::MatrixXd::Identity(3, 3);
                damping.setZero();
                damping.topLeftCorner(3, 3) << 2.0 * sqrt(K_trans) * Eigen::MatrixXd::Identity(3, 3);
                damping.bottomRightCorner(3, 3) << 2.0 * sqrt(K_rot) *Eigen::MatrixXd::Identity(3, 3);

                // Nullspace parameters same as the compliance parameters
                Eigen::MatrixXd null_stiffness(7, 7), null_damping(7, 7);
                null_stiffness.setZero();
                null_damping.setZero();
                null_stiffness.topLeftCorner(3, 3) = k_nullspace_trans * Eigen::Matrix3d::Identity();
                null_stiffness.bottomRightCorner(4, 4) = k_nullspace_rot * Eigen::MatrixXd::Identity(4, 4);

                //set load
                franka_robot.setLoad(0.2, {0.0, 0.0, 0.1},                          
                {0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0}
                );

                // Load the robot model
                franka::Model model(franka_robot.loadModel());

                // start the motion with Cartesian impedance control
                uint32_t motion_id = franka_robot.startMotion(
                    research_interface::robot::Move::ControllerMode::kExternalController,
                    research_interface::robot::Move::MotionGeneratorMode::kJointVelocity,
                    deviation, deviation
                );

                // Read the robot state once
                franka::RobotState robot_state = franka_robot.readOnce();

                // equilibrium point is the initial position
                Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
                Eigen::Vector3d position_d(initial_transform.translation());
                Eigen::Quaterniond orientation_d(initial_transform.rotation());

                // Define the desired joint configuration
                Eigen::VectorXd q_d(7);
                q_d << robot_state.q[0], robot_state.q[1], robot_state.q[2],
                    robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6];

                franka::Duration previous_time = robot_state.time;
                franka::Duration period;
                double time = 0.0;
                double kDeltaT = 1e-3;

                // compute control
                Eigen::VectorXd tau_task(7), tau_d(7), tau_nullspace(7);

                while (!motion_command.motion_generation_finished) {

                    auto t_start = std::chrono::high_resolution_clock::now();

                    // Update time
                    period = robot_state.time - previous_time;
                    previous_time = robot_state.time;
                    time += period.toSec();

                    // get state variables
                    std::array<double, 7> coriolis_array = model.coriolis(robot_state);
                    std::array<double, 42> jacobian_array =model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                    std::array<double, 7> gravity_array = model.gravity(robot_state);
                    Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

                    // convert to Eigen
                    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
                    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
                    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
                    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
                    Eigen::Vector3d position(transform.translation());
                    Eigen::Quaterniond orientation(transform.rotation());
                    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(7, 7);
                    Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().solve(Eigen::MatrixXd::Identity(6, 6));
                    Eigen::MatrixXd nullspace_projector = identity - jacobian.transpose() * jacobian_pinv.transpose();

                    // compute error to desired equilibrium pose
                    // position error
                    Eigen::Matrix<double, 6, 1> error;
                    error.head(3) << position - position_d;

                    // orientation error
                    // "difference" quaternion
                    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                        orientation.coeffs() << -orientation.coeffs();
                    }
                    // "difference" quaternion
                    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
                    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                    // Transform to base frame
                    error.tail(3) << -transform.rotation() * error.tail(3);

                    // Spring damper system with damping ratio=1
                    tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
                    //print tau_nullspace elements
                    tau_nullspace = nullspace_projector * (null_stiffness * (q_d - q) - null_damping * dq);
                    tau_d << tau_task + tau_nullspace + coriolis;
                    
                    //pass tau_d to the control command
                    control_command.tau_J_d.fill(0.0);
                    for (size_t i = 0; i < 7; ++i) {
                        control_command.tau_J_d[i] = tau_d[i];
                    }

                    // Apply low-pass filter to the torque command
                    for (size_t i = 0; i < 7; ++i) {
                        control_command.tau_J_d[i] = franka::lowpassFilter(
                            kDeltaT,
                            control_command.tau_J_d[i],
                            robot_state.tau_J_d[i],
                            franka::kDefaultCutoffFrequency
                        );
                    }

                    //rate limit the torque command
                    control_command.tau_J_d = franka::limitRate(
                        franka::kMaxTorqueRate,
                        control_command.tau_J_d,
                        robot_state.tau_J_d
                    );

                    // Send command, receive updated state
                    robot_state = franka_robot.update(&motion_command, &control_command);
                    franka_robot.throwOnMotionError(robot_state, motion_id);

                    auto t_end = std::chrono::high_resolution_clock::now();
                    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
                    std::cout << "setTorques time: " << elapsed_ms << " ms" << std::endl;

                    // Stop after 30 seconds
                    if (time >= 3600.0) {
                        motion_command.motion_generation_finished = true;
                    }

                }

                // Finish motion
                franka_robot.finishMotion(motion_id, &motion_command, &control_command);
                std::cout << "[Test 8] Motion session finished successfully." << std::endl;

                // Final state
                franka::RobotState final_state = franka_robot.readOnce();
                std::cout << "[Test 8] Robot state after motion: " << final_state.q << std::endl;

            } catch (const std::exception& e) {
                std::cerr << "[Test 8] Cartesian motion error: " << e.what() << std::endl;
                return -1;
            }

            break;
        
        case 9:
            std::cout << "[Test 9] test libfranka readOnce() and RobotInterfaces functions" << std::endl;

            try
            {
                // variables for get functions
                RUT::VectorXd joint_positions;
                RUT::VectorXd joint_torques;
                RUT::Vector7d cartesian_pose;

                //variables for franka reads
                RUT::VectorXd q_d(7);
                RUT::VectorXd tau_d(7);
                RUT::VectorXd pose(7);

                // first compare readOnce().q vs FRANKA::getJoints()
                franka::RobotState robot_state = franka_robot.readOnce();
        
                // save robot_state.q to q_d
                for (size_t i = 0; i < 7; ++i) {
                    q_d[i] = robot_state.q[i];
                }
                std::cout << "[Test 9] Robot state readOnce().q: " << q_d << std::endl;
                
                //get joints with RobotInterface
                franka_robot.getJoints(joint_positions);
                std::cout << "[Test 9] Robot state getJoints: " << joint_positions << std::endl;

                //compare readOnce().O_T_EE vs FRANKA::getCartesianPose()
                // convert robot_state.O_T_EE (array 16) to pose (array 7)
                Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
                Eigen::Vector3d position(transform.translation());
                Eigen::Quaterniond orientation(transform.rotation());
                pose[0] = position[0];
                pose[1] = position[1];
                pose[2] = position[2];
                pose[3] = orientation.x();
                pose[4] = orientation.y();
                pose[5] = orientation.z();
                pose[6] = orientation.w();
                std::cout << "[Test 9] Robot state readOnce().O_T_EE: " << pose << std::endl;

                //get cartesian pose with RobotInterface
                franka_robot.getCartesian(cartesian_pose);
                std::cout << "[Test 9] Robot state getCartesianPose: " << cartesian_pose << std::endl;

                //finally compare readOnce().tau_J_d vs FRANKA::getTorques()
                for (size_t i = 0; i < 7; ++i) {
                    tau_d[i] = robot_state.tau_J_d[i];
                }
                std::cout << "[Test 9] Robot state readOnce().tau_J_d: " << tau_d << std::endl;

                //get torques with RobotInterface
                franka_robot.getTorques(joint_torques);
                std::cout << "[Test 9] Robot state getTorques: " << joint_torques << std::endl;

            }
            catch (const std::exception& e) {
                std::cerr << "[Test 9] Get/Set functions error: " << e.what() << std::endl;
                return -1;
            }
            break;

            case 10:
            std::cout << "[Test 10] set joint position test using libfranka " << std::endl;

            try {

                //set impedance to robot
                franka_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
                franka_robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
                franka_robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});


                uint32_t motion_id = franka_robot.startMotion(
                    research_interface::robot::Move::ControllerMode::kJointImpedance,
                    research_interface::robot::Move::MotionGeneratorMode::kJointPosition,
                    deviation, deviation
                );

                // Read the initial robot state
                //franka::RobotState init_state = franka_robot.readOnce();
                franka::RobotState robot_state = franka_robot.update(nullptr, nullptr);
                franka_robot.throwOnMotionError(robot_state, motion_id);
                std::cout << "[Test 5] Robot state before motion: " << robot_state.q << std::endl;

                // Initial setup: capture initial state
                std::array<double, 7> target_position = robot_state.q;

                // franka duration for the control loop
                franka::Duration period;
                franka::Duration previous_time = robot_state.time;

                double time = 0.0;
                double kDeltaT = 1e-3;

                while(!motion_command.motion_generation_finished) {

                    // calculate the time step
                    period = robot_state.time - previous_time;
                    previous_time = robot_state.time;
                    time += period.toSec();

                    // Update target position based on time
                    double delta = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));

                    // here goes SpinMotion similar part
                    //move joint 5
                    motion_command.q_c = target_position;
                    motion_command.q_c[4] = motion_command.q_c[4] + delta;
                    
                    // Apply low-pass filter from lowpass_filter.h to the target position
                    for (size_t i = 0; i < 7; ++i) {
                        motion_command.q_c[i] = franka::lowpassFilter(
                            kDeltaT,
                            motion_command.q_c[i],
                            robot_state.q_d[i],
                            franka::kDefaultCutoffFrequency
                        );
                    }
                    
                    //limit rate of the motion command
                    motion_command.q_c = franka::limitRate(
                        franka::kMaxJointVelocity,
                        franka::kMaxJointAcceleration,
                        franka::kMaxJointJerk,
                        motion_command.q_c,
                        robot_state.q_d,
                        robot_state.dq_d,
                        robot_state.ddq_d
                    );

                    // Update robot: send command and receive new state
                    robot_state = franka_robot.update(&motion_command, nullptr);
                    franka_robot.throwOnMotionError(robot_state, motion_id);
                    
                    // if time is 5
                    if (time >= 5.0) {
                        motion_command.motion_generation_finished = true;
                    }

                }

                std::cout << "[Test 5] Motion command finished." << std::endl;
                // Finish the motion session (deberia meter en el while con un flag cuando termina el movimiento)
                franka_robot.finishMotion(motion_id, &motion_command, nullptr);

                // Read final state
                franka::RobotState final_state = franka_robot.readOnce();
                std::cout << "[Test 5] Robot state after motion: " << final_state.q << std::endl;

            } catch (const std::exception& e) {
                std::cerr << "[Test 5] Failed to start/finish motion: " << e.what() << std::endl;
                return -1;
            }
            break;
            

        default:
            std::cout << "Invalid test case selected." << std::endl;
            break;
    }

    return 0;
}