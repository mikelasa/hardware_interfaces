#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include "ur_rtde/ur_rtde.h"

int main() {
  // Create and configure the UR robot connection parameters
  URRTDE::URRTDEConfig config;
  config.robot_ip = "192.168.2.105";
  config.rtde_frequency = 500;
  config.rt_receive_priority = 90;
  config.rt_control_priority = 85;
  config.interface_priority = 80;
  config.linear_vel = 0.5;
  config.linear_acc = 0.5;
  config.servoL_lookahead_time = 0.1;
  config.servoL_gain = 600;
  // Set safety and operation modes for the robot
  config.robot_interface_config.zone_safety_mode =
      RobotSafetyMode::SAFETY_MODE_TRUNCATE;
  config.robot_interface_config.incre_safety_mode =
      RobotSafetyMode::SAFETY_MODE_STOP;
  config.robot_interface_config.operation_mode =
      RobotOperationMode::OPERATION_MODE_CARTESIAN;
  config.robot_interface_config.max_incre_m = 0.002;      // 1 m per second
  config.robot_interface_config.max_incre_rad = 0.00628;  // 3.14 per second
  config.robot_interface_config.safe_zone = {0.3, 0.65, -0.3, 0.4, 0.1, 0.4};

  // Instantiate the URRTDE robot interface
  URRTDE ur_rtde;
  // Timer for control loop timing
  RUT::Timer timer;
  // Initialize the robot connection and communication
  ur_rtde.init(timer.tic(), config);

  // Get the initial Cartesian pose of the robot
  RUT::Vector7d pose0;
  ur_rtde.getCartesian(pose0);
  RUT::Vector7d pose_ref = pose0;

  // Start the timer for the control loop
  timer.tic();
  while (true) {
    // Mark the start of the control period
    RUT::TimePoint t_start = ur_rtde.rtde_init_period();

    // Get elapsed time in milliseconds
    double dt = timer.toc_ms();
    // Print current time and pose
    printf("t = %f, pose: %f %f %f %f %f %f %f\n", dt, pose_ref[0], pose_ref[1],
           pose_ref[2], pose_ref[3], pose_ref[4], pose_ref[5], pose_ref[6]);

    // Generate a small trajectory in X and Y using sin/cos functions
    double dx = 0.05 * (1.0 - cos(1.2 * dt / 1000.0));
    double dy = 0.05 * sin(1.2 * dt / 1000.0);
    pose_ref[0] = pose0[0] + dx;
    pose_ref[1] = pose0[1] + dy;
    
    // Send the new pose to the robot in real time
    if (!ur_rtde.streamCartesian(pose_ref)) {
      printf("streamCartesian failed\n");

      break;
    }

    // Stop after 30 seconds
    if (dt > 30000)
      break;

    // Wait for the control period to finish to maintain loop frequency
    ur_rtde.rtde_wait_period(t_start);
  }

  // End of test
  return 0;
}