#include <unistd.h>
#include <csignal>
#include <iostream>
#include <mutex>
#include <fstream>
#include <vector>
#include <limits>
#include <filesystem>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include <table_top_manip/manip_server.h>

namespace fs = std::filesystem;

// Structure to hold robot trajectory data
struct RobotTrajectoryPoint {
  int seq_id;
  int mask;
  double robot_time_stamps;
  RUT::Vector7d ts_pose_fb;
};

// Function to get list of episode folders in a directory
std::vector<std::string> get_episode_folders(const std::string& data_folder) {
  std::vector<std::string> episodes;
  
  if (!fs::exists(data_folder) || !fs::is_directory(data_folder)) {
    std::cerr << "Data folder does not exist: " << data_folder << std::endl;
    return episodes;
  }
  
  for (const auto& entry : fs::directory_iterator(data_folder)) {
    if (entry.is_directory()) {
      std::string folder_name = entry.path().filename().string();
      // Check if folder name starts with "episode_"
      if (folder_name.find("episode_") == 0) {
        episodes.push_back(entry.path().string());
      }
    }
  }
  
  // Sort episodes by name (which includes timestamp, so newest last)
  std::sort(episodes.begin(), episodes.end());
  
  return episodes;
}

// Function to select an episode interactively
std::string select_episode(const std::string& data_folder) {
  std::vector<std::string> episodes = get_episode_folders(data_folder);
  
  if (episodes.empty()) {
    std::cerr << "No episodes found in: " << data_folder << std::endl;
    return "";
  }
  
  std::cout << "\n========== Available Episodes ==========" << std::endl;
  for (size_t i = 0; i < episodes.size(); ++i) {
    std::string folder_name = fs::path(episodes[i]).filename().string();
    std::cout << "  [" << i << "] " << folder_name << std::endl;
  }
  std::cout << "========================================" << std::endl;
  std::cout << "Select episode number (0-" << (episodes.size() - 1) << "): ";
  
  int selection;
  std::cin >> selection;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  
  if (selection < 0 || selection >= static_cast<int>(episodes.size())) {
    std::cerr << "Invalid selection." << std::endl;
    return "";
  }
  
  return episodes[selection];
}

// Function to load trajectory from JSON file
bool load_trajectory_from_json(const std::string& filepath, 
                                std::vector<RobotTrajectoryPoint>& trajectory) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Failed to open trajectory file: " << filepath << std::endl;
    return false;
  }

  try {
    YAML::Node data = YAML::Load(file);
    trajectory.clear();
    trajectory.reserve(data.size());

    for (const auto& point : data) {
      RobotTrajectoryPoint traj_point;
      traj_point.seq_id = point["seq_id"].as<int>();
      traj_point.mask = point["mask"].as<int>();
      traj_point.robot_time_stamps = point["robot_time_stamps"].as<double>();
      
      auto pose_vec = point["ts_pose_fb"].as<std::vector<double>>();
      if (pose_vec.size() != 7) {
        std::cerr << "Invalid pose size at seq_id " << traj_point.seq_id << std::endl;
        return false;
      }
      
      for (int i = 0; i < 7; ++i) {
        traj_point.ts_pose_fb[i] = pose_vec[i];
      }
      
      trajectory.push_back(traj_point);
    }

    std::cout << "Loaded " << trajectory.size() << " trajectory points from " 
              << filepath << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    return false;
  }
}

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

    // Hardcoded data folder path
      const std::string data_folder = "/home/robotlab/data/real/test_3";
        ///home/robotlab/data/real/test_fuerzas
        //"/home/robotlab/data/real/draw_single_arm"

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

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Main loop
  RUT::Timer duration_timer;
  while (true) {
    main_print("Choose mode:");
    std::cout << "  [1] Manual recording (free jogging)" << std::endl;
    std::cout << "  [2] Replay trajectory and record" << std::endl;
    std::cout << "  [q] Quit" << std::endl;
    std::cout << "Enter choice: ";
    
    char mode_choice = std::getchar();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    if (mode_choice == 'q') {
      std::cout << "[main]: Quitting the program." << std::endl;
      break;
    }
    
    if (mode_choice == '1') {
      // ========== MANUAL RECORDING MODE ==========
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

      std::cout << "[main] Episode saved and continuing." << std::endl;
      
    } else if (mode_choice == '2') {
      // ========== REPLAY ALL EPISODES MODE ==========
      
      std::cout << "[main] Using data folder: " << data_folder << std::endl;
      
      // Get all episode folders
      std::vector<std::string> episodes = get_episode_folders(data_folder);
      if (episodes.empty()) {
        std::cerr << "[main] No episodes found. Returning to menu." << std::endl;
        continue;
      }
      
      std::cout << "[main] Found " << episodes.size() << " episodes to replay." << std::endl;
      main_print("Press Enter to start replaying all episodes.");
      std::getchar();
      
      // Loop through all episodes
      for (size_t ep_idx = 0; ep_idx < episodes.size(); ++ep_idx) {
        std::string episode_folder = episodes[ep_idx];
        std::string episode_name = fs::path(episode_folder).filename().string();
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "[main] Episode " << (ep_idx + 1) << "/" << episodes.size() 
                  << ": " << episode_name << std::endl;
        std::cout << "========================================" << std::endl;
        
        // Construct path to robot_data_0.json
        std::string json_path = episode_folder + "/robot_data_0.json";
        
        // Load trajectory
        std::vector<RobotTrajectoryPoint> trajectory;
        if (!load_trajectory_from_json(json_path, trajectory)) {
          std::cerr << "[main] Failed to load trajectory. Skipping this episode." << std::endl;
          continue;
        }
        
        if (trajectory.empty()) {
          std::cerr << "[main] Trajectory is empty. Skipping this episode." << std::endl;
          continue;
        }
        
        // First, move to the starting pose WITHOUT recording
        std::cout << "[main] Moving to start position (not recording)..." << std::endl;
        RUT::Timer move_to_start_timer;
        move_to_start_timer.set_loop_rate_hz(1000);
        
        const double MOVE_TO_START_TIME_MS = 3000.0; // 3 seconds to move to start
        while (move_to_start_timer.toc_ms() < MOVE_TO_START_TIME_MS) {
          double alpha = move_to_start_timer.toc_ms() / MOVE_TO_START_TIME_MS;
          
          // Get current pose
          Eigen::MatrixXd cur_pose_m = server.get_pose(1);
          if (cur_pose_m.rows() >= 1 && cur_pose_m.cols() == 7) {
            RUT::Vector7d cur_pose;
            for (int i = 0; i < 7; ++i) cur_pose[i] = cur_pose_m(0, i);
            
            // Linear interpolation from current to start pose
            RUT::Vector7d intermediate_pose = cur_pose + alpha * (trajectory[0].ts_pose_fb - cur_pose);
            server.set_target_pose(intermediate_pose, 10.0, 0);
          }
          
          move_to_start_timer.sleep_till_next();
        }
        
        // Send the exact start pose
        server.set_target_pose(trajectory[0].ts_pose_fb, 10.0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // Brief settle time
        
        // Now start recording
        std::cout << "[main] Starting to record trajectory..." << std::endl;
        server.start_saving_data_for_a_new_episode();
        std::cout << "[main] Replaying trajectory with " << trajectory.size() 
                  << " points..." << std::endl;
        
        // Replay the trajectory
        duration_timer.tic();
        RUT::Timer replay_timer;
        replay_timer.set_loop_rate_hz(1000); // 1kHz replay rate to match recording rate
        
        double start_time_ms = trajectory[0].robot_time_stamps;
        
        for (size_t i = 0; i < trajectory.size(); ++i) {
          double elapsed_ms = replay_timer.toc_ms();
          double target_time_ms = trajectory[i].robot_time_stamps - start_time_ms;
          
          // Wait until we reach the target time
          while (elapsed_ms < target_time_ms) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            elapsed_ms = replay_timer.toc_ms();
          }
          
          // Send the pose command
          server.set_target_pose(trajectory[i].ts_pose_fb, 10.0, 0); // 10ms lookahead
          
          replay_timer.sleep_till_next();
        }
        
        double episode_duration_s = duration_timer.toc_ms() / 1000.0;
        std::cout << "[main] Trajectory replay finished. Duration: " 
                  << episode_duration_s << "s" << std::endl;
        
        // Hold final position
        server.set_high_level_maintain_position();
        
        // Stop saving data
        server.stop_saving_data();
        while (server.is_saving_data()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "[main] Episode " << (ep_idx + 1) << "/" << episodes.size() 
                  << " saved successfully." << std::endl;
        
        // Wait for user input before starting next episode
        std::cout << "[main] Press Enter to start the next episode..." << std::endl;
        std::getchar();
      }
      
      std::cout << "\n========================================" << std::endl;
      std::cout << "[main] All " << episodes.size() << " episodes replayed!" << std::endl;
      std::cout << "========================================\n" << std::endl;
      
    } else {
      std::cout << "[main] Invalid choice. Please try again." << std::endl;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
  }
  // join the threads
  server.join_threads();

  std::cout << "[main]: Threads have joined. Exiting." << std::endl;
  return 0;
}
