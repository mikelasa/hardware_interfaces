#include <unistd.h>
#include <csignal>
#include <iostream>
#include <mutex>

#include <yaml-cpp/yaml.h>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include <table_top_manip/manip_server.h>

/* 
data/
  episode_xxxx/
    rgb/
      0000.jpg
      0001.jpg
      ...
    low_dim_data.json
*/

void main_print(const std::string& msg) {
  std::cout << "================================================" << std::endl;
  std::cout << "== Main Stage" << std::endl;
  std::cout << "== " << msg << std::endl;
  std::cout << "================================================" << std::endl;
}

int main() {
  
  /*
  // read config files, loads:
      - main params: is bimanual, data folder...
      - config for what threads to run and parameters
      - hardware config (robot, camera...)
      -  controller config
  */
  const std::string config_path =
      "/home/robotlab/ACP/hardware_interfaces/workcell/"
      "table_top_manip/"
      "config/single_arm_data_collection_franka.yaml";


  // create the server, this server handles all the data collection pipeline, communication with hardware, etc...
  ManipServer server(config_path);

  /*
    wait for threads to be ready:
      - RGB thread
      - robot
      - gripper (if gripper)
      - wrench (if sensor)

    if controller is impedance controller, the wrench buffer needs some time to fill up
  */ 
  while (!server.is_ready()) {
    std::cout << "Waiting for server to be ready." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
  }

  // use all dofs for compliance (not used in impedance controller)
  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  int n_af = 6;
  // tells the admittance controller which directions are compliant to external force (solo en admitance)
  server.set_force_controlled_axis(Tr, n_af, 0);
  // if bimanual, set the second arm's force controlled axis
  if (server.is_bimanual()) {
    server.set_force_controlled_axis(Tr, n_af, 1);
  }

  // use high stiffness to hold position being compliant
  server.set_high_level_maintain_position();

  // Main loop
  RUT::Timer duration_timer;
  while (true) {
    main_print("Press Enter to start a new episode.");
    std::getchar();
    duration_timer.tic();

    // set the robot to be compliant, stiffness low
    server.set_high_level_free_jogging();

    // start saving data 
    main_print(
        "[main] Recording in progress. Press Enter to finish the episode.");
    server.start_saving_data_for_a_new_episode();

    // wait for user to stop the episode
    std::getchar();

    // set the robot to hold position with high stiffness
    server.set_high_level_maintain_position();
    double episode_duration_s = duration_timer.toc_ms() / 1000.0;
    std::cout << "[main] Episode finished with duration " << episode_duration_s
              << "s. Waiting for threads to stop saving." << std::endl;

    // stop saving data
    server.stop_saving_data();
    while (server.is_saving_data()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[main] All threads have stopped saving data." << std::endl;

    std::cout << "[main] What to do:\n";
    std::cout << "[main]    q to quit\n";
    std::cout << "[main]    others to save and continue.\n";
    char c = std::getchar();

    if (c == 'q') {
      std::cout << "[main]: Quitting the program." << std::endl;
      break;
    } else {
      std::cout << "[main]: Saving and continuing." << std::endl;
    }
  }
  // join the threads
  server.join_threads();

  std::cout << "[main]: Threads have joined. Exiting." << std::endl;
  return 0;
}
