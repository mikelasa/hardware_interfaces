#include "franka/franka.h"
#include "robot_impl.h"

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
    config.tcp_inertia = 0.01;
    config.cartesian_impedance = {300, 300, 300, 30, 30, 30};
    config.deviation = {0.05, 0.05, 0.5}; // Example deviation values

    research_interface::robot::Move::Deviation deviation{
        config.deviation[0],
        config.deviation[1],
        config.deviation[2]
    };

    FRANKA franka_robot(config);
    research_interface::robot::MotionGeneratorCommand motion_command{};
    research_interface::robot::ControllerCommand control_command{};
    
    int test_case = 4; // Change this to test different functionalities

    // create a switch case for different tests for robot
    switch (test_case) {
        case 1:
            // Test 1: Get Robot State
            //instancia la clase FRANKA con la configuracion
            try {
                FRANKA franka_robot(config);
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
                FRANKA franka_robot(config);
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

                std::this_thread::sleep_for(std::chrono::seconds(4));  // wait 1s just to observe session

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

                // Simulate some control command
                control_command.tau_J_d.fill(0.1); // Example torque values

                std::this_thread::sleep_for(std::chrono::seconds(2));  // wait 2s just to observe session

                franka_robot.finishMotion(motion_id, nullptr, &control_command);
                std::cout << "[Test 4] Motion session finished successfully." << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[Test 4] Failed to start/finish motion: " << e.what() << std::endl;
                return -1;
            }
        }
         
        default:
            std::cout << "Invalid test case selected." << std::endl;
            break;
    }

    return 0;
}