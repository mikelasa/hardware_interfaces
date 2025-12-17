#include "table_top_manip/manip_server.h"

#include <RobotUtilities/interpolation_controller.h>
#include <opencv2/core/eigen.hpp>

#include "helpers.hpp"

// robot loop implementation for impedance controller
void ManipServer::robot_impedance_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][Robot Impedance thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  // Set real-time priority
  if (!_config.mock_hardware) {
    struct sched_param param;
    param.sched_priority = 99;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
      std::cerr << header << "Failed to set real-time priority: " 
                << strerror(errno) << std::endl;
      std::cerr << header << "Try running with sudo or setting CAP_SYS_NICE capability" 
                << std::endl;
    } else {
      std::cout << header << "Real-time priority set successfully" << std::endl;
    }
  }

  /* --------- VARIABLES --------- */
  // using the global timer, creates a local timer for this thread
  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::Vector7d pose_fb; // current pose feedback
  RUT::Vector6d vel_fb; // current velocity feedback
  RUT::Vector7d pose_target_waypoint; // target pose waypoint
  RUT::Vector7d ref_pose; //reference pose
  RUT::Vector7d torque_robot_cmd; // torque command for robot (step output)

  // The following two initial values are used in mock hardware mode
  pose_fb << id, 0, 0, 1, 0, 0, 0;
  torque_robot_cmd = RUT::Vector7d::Zero();
  vel_fb << 0, 0, 0, 0, 0, 0;

  RUT::Vector6d wrench_fb_ur, wrench_WTr; // current wrench feedback and transformed wrench world to tool frame
  RUT::Matrix6d stiffness;

  /* --------- INITIALIZATION (ROBOT, INTERPOLATOR, PROFILER) --------- */

  // pointer to the robot, to access specific functions
  FRANKA* franka_ptr;

  // Get FRANKA pointer and get initial pose
  if (!_config.mock_hardware) {
    franka_ptr = static_cast<FRANKA*>(robot_ptrs[id].get());
    franka_ptr->getCartesian(pose_fb);
  }

   // initialize jacobain, state and model for franka
  franka::Model model = franka::Model(franka_ptr->loadModel());
  franka::RobotState state = franka_ptr->getRobotState();
  std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // set initial values for force control
  ref_pose = pose_fb;
  wrench_WTr.setZero();

  //A controller that interpolates linearly between two Cartesian targets.
  // initializes with the current pose and timestamp
  RUT::TaskSpaceInterpolationController intp_controller;
  intp_controller.initialize(pose_fb, timer.toc_ms());
  std::cout << header << "intp_controller initialized with pose_fb: "
            << pose_fb.transpose() << std::endl;

  // this part sets the robot thread state to ready (initialize in manip server)
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_robot_thread_ready[id] = true;
  }

  // profile measures the loop execution time and performance (important for real time control)
  RUT::Profiler loop_profiler;
  std::cout << header << "Loop started." << std::endl;

  // Statistics tracking for overruns
  uint64_t loop_count = 0;
  uint64_t overrun_count = 0;
  double max_overrun_ms = 0.0;
  double total_overrun_ms = 0.0;
  
  // Track Franka's actual success rate
  double last_franka_success_rate = 1.0;
  uint64_t last_success_rate_check = 0;
  
  // Track waypoint starvation
  uint64_t no_waypoint_count = 0;
  uint64_t consecutive_holds = 0;
  uint64_t max_consecutive_holds = 0;

  RUT::Timer mock_loop_timer;
  // Control loop at 1kHz for franka, can be changed if needed
  mock_loop_timer.set_loop_rate_hz(1000);
  mock_loop_timer.start_timed_loop();

  /* --------- CONTROL LOOP --------- */

  while (true) {
    // Update robot status
    loop_profiler.start();
    RUT::TimePoint t_start;
    double time_now_ms;
    if (!_config.mock_hardware) {
      /* --------- UPDATE STATES --------- */
      // updates robot states
      franka_ptr->getCurrentPose(pose_fb);
      franka_ptr->getCurrentWrenchTool(wrench_fb_ur);
      //franka_ptr->getCartesianVelocity(vel_fb);
      state = franka_ptr->getRobotState();
      jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

      //update controller states for jacobian
      _impedance_controllers[id].getJacobian(Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data()));
      _impedance_controllers[id].getRobotState(state);

      // map the array to eigen
      time_now_ms = timer.toc_ms();

      // save feedback in buffers
      loop_profiler.stop("compute");
      loop_profiler.start();
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        _poses_fb[id] = pose_fb;
      }
      loop_profiler.stop("lock");
      loop_profiler.start();

    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      wrench_fb_ur.setZero();
    }
    /* --------- UPDATE BUFFERS --------- */
    // buffer robot pose and wrench (save data)
    loop_profiler.stop("compute");
    loop_profiler.start();
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      _pose_buffers[id].put(pose_fb);
      _pose_timestamp_ms_buffers[id].put(time_now_ms);
    }
    //wrench buffer
    {
      std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
      _robot_wrench_buffers[id].put(wrench_fb_ur);
      _robot_wrench_timestamp_ms_buffers[id].put(time_now_ms);
    }
    loop_profiler.stop("lock");
    loop_profiler.start();

    /* --------- UPDATE INTERPOLATOR --------- */
    // update  target from interpolation controller
    // get_control returns false if no valid target is found, if so, needs to create a new one
    if (!intp_controller.get_control(time_now_ms, ref_pose)) {
      bool new_wp_found = false;
      {
        // need to get new waypoint from buffer
        std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
        while (!_waypoints_buffers[id].is_empty()) {
          // keep querying buffer until we get a target that is in the future
          pose_target_waypoint = _waypoints_buffers[id].pop();
          double target_time_ms = _waypoints_timestamp_ms_buffers[id].pop();
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(pose_target_waypoint,
                                           target_time_ms);
            new_wp_found = true;
            break;
          }
        }
      }
      if (!new_wp_found) {
        // std::cout << "[debug] time_now_ms: " << time_now_ms
        //           << ", time now: " << timer.toc_ms()
        //           << ", target_time_ms:" << target_time_ms
        //           << ", pose_target_waypoint: "
        //           << pose_target_waypoint.transpose() << std::endl;
        intp_controller.keep_the_last_target(time_now_ms);
      }
      intp_controller.get_control(time_now_ms, ref_pose);
    }

    loop_profiler.stop("intp_controller");
    loop_profiler.start();

    /* --------- UPDATE STIFFNESS --------- */
    // update stiffness matrix from buffer
    // condition:
    //   time_now_ms < time[0], do nothing
    //   time_now_ms >= time[0], look for next
    bool new_stiffness_found = false;
    {
      std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
      if (!_stiffness_buffers[id].is_empty()) {
        double next_available_time_ms = _stiffness_timestamp_ms_buffers[id][0];
        if (time_now_ms > next_available_time_ms) {
          new_stiffness_found = true;
          while ((!_stiffness_timestamp_ms_buffers[id].is_empty()) &&
                 (_stiffness_timestamp_ms_buffers[id][0] < time_now_ms)) {
            stiffness = _stiffness_buffers[id].pop();
            next_available_time_ms = _stiffness_timestamp_ms_buffers[id].pop();
          }
        }
      }
    }
    loop_profiler.stop("stiffness");
    loop_profiler.start();

    wrench_WTr.setZero();

    /* --------- UPDATE CONTROLLER --------- */
    // std::cout << "[debug] time: " << time_now_ms
    //           << ", wrench_fb_ur: " << wrench_fb_ur.transpose()
    //           << ", wrench_WTr: " << wrench_WTr.transpose() << std::endl;

    // Update the compliance controller
    {
      std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
      loop_profiler.stop("controller_lock");
      loop_profiler.start();
      _impedance_controllers[id].setRobotStatus(pose_fb, wrench_fb_ur);
      // Update robot reference
      _impedance_controllers[id].setRobotReference(ref_pose, wrench_WTr);

      // Update stiffness matrix
      if (new_stiffness_found) {
        _impedance_controllers[id].setStiffnessMatrix(stiffness);
      }
      loop_profiler.stop("controller_set");
      loop_profiler.start();
      // Compute the control output
      _impedance_controllers[id].step(torque_robot_cmd);
      loop_profiler.stop("controller_step");

    }

    // Send control command to the robot
    loop_profiler.start();
    if ((!_config.mock_hardware) &&
        (!franka_ptr->setTorques(torque_robot_cmd))) {
      std::cout << header << "setTorques failed. Ending thread."
                << std::endl;
      std::cout << header << "last pose_fb: " << pose_fb.transpose()
                << std::endl;
      std::cout << header << "last wrench_fb_ur: " << wrench_fb_ur.transpose()
                << std::endl;
      std::cout << header << "last ref_pose: "
                << ref_pose.transpose() << std::endl;
      std::cout << header << "last torque : " << torque_robot_cmd.transpose()
                << std::endl;
      break;
    }
    loop_profiler.stop("franka_update");

    // std::cout << "t = " << timer.toc_ms()
    //           << ", pose_rdte_cmd: " << pose_rdte_cmd.transpose() << std::endl;

    // logging - just push to buffer (ultra-fast, <10 microseconds)
    loop_profiler.start();
    if (_ctrl_flag_saving) {
      std::lock_guard<std::mutex> lock(_logging_buffer_mtxs[id]);
      
      if (!_logging_buffers[id].is_full()) {
        RobotLogData log_data;
        log_data.timestamp_ms = time_now_ms;
        log_data.pose_fb = pose_fb;
        // Extract joint torques from robot state
        for (int i = 0; i < 7; ++i) {
          log_data.tau_J[i] = state.tau_J[i];
          log_data.q[i] = state.q[i];  // joint positions
        }
        
        _logging_buffers[id].put(log_data);
        _states_robot_thread_saving[id] = true;
        _logging_buffer_overflow[id].store(false);
      } else {
        // Buffer overflow - logging thread can't keep up
        if (!_logging_buffer_overflow[id].load()) {
          _logging_buffer_overflow[id].store(true);
          std::cerr << header << "WARNING: Logging buffer overflow!" << std::endl;
        }
      }
    } else {
      _states_robot_thread_saving[id] = false;
    }
    loop_profiler.stop("logging");

    // loop control
    loop_profiler.start();
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[robot thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }

    loop_profiler.stop("lock");

    // loop timing and overrun check
    loop_count++;
    if (_config.mock_hardware) {
      mock_loop_timer.sleep_till_next();
    } else {
      // Check Franka's actual success rate every 1000 loops (~1 second)
      if (loop_count % 1000 == 0) {
        last_franka_success_rate = state.control_command_success_rate;
        
        // Only print if there's an actual problem
        if (last_franka_success_rate < 0.95) {
          std::cout << "\033[31m";  // red
          std::cout << header << "⚠️ WARNING: Franka success rate: " 
                    << (last_franka_success_rate * 100.0) << "% "
                    << "(Local overruns: " << (100.0 * overrun_count / loop_count) << "%)"
                    << std::endl;
          std::cout << "\033[0m";
        } else {
          // Success rate is good - only print occasionally (every 10 seconds)
          if (loop_count % 10000 == 0) {
            std::cout << "\033[32m";  // green
            std::cout << header << "✅ Franka Success Rate: " 
                      << (last_franka_success_rate * 100.0) << "%" << std::endl;
            std::cout << "\033[0m";
          }
        }
      }
      
      double overrun_ms = mock_loop_timer.check_for_overrun_ms(false);
      if (overrun_ms > 0) {
        overrun_count++;
        total_overrun_ms += overrun_ms;
        if (overrun_ms > max_overrun_ms) {
          max_overrun_ms = overrun_ms;
        }
        
        // Don't print overrun warnings - they're harmless as long as Franka success rate is high
        // Statistics will be shown at the end
      }
      mock_loop_timer.check_for_overrun_ms(
          false);  // just call it to reset the timer
    }
    loop_profiler.clear();
  }  // end of while loop

  // Print final impedance loop statistics
  std::cout << "\033[36m";  // cyan color
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << "📈 FINAL CONTROL LOOP STATISTICS:" << std::endl;
  std::cout << header << "   Total loops: " << loop_count << std::endl;
  std::cout << header << "   Overruns: " << overrun_count << " (" 
            << (100.0 * overrun_count / loop_count) << "%)" << std::endl;
  if (overrun_count > 0) {
    std::cout << header << "   Avg overrun: " << (total_overrun_ms / overrun_count) << " ms" << std::endl;
    std::cout << header << "   Max overrun: " << max_overrun_ms << " ms" << std::endl;
  }
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\033[0m";  // reset color

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[robot thread] Joined." << std::endl;
}

//--------ADMITTANCE CONTROLLER LOOP ---------//
void ManipServer::robot_admittance_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][Robot Admittance thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  // using the global timer, creates a local timer for this thread
  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::Vector7d pose_fb; // current pose feedback
  RUT::Vector7d pose_target_waypoint; // target pose waypoint
  RUT::Vector6d vel_fb; // current velocity feedback
  RUT::Vector7d force_control_ref_pose; // force control reference pose
  RUT::Vector7d pose_rdte_cmd; // pose command for RTDE
  // The following two initial values are used in mock hardware mode
  pose_fb << id, 0, 0, 1, 0, 0, 0;
  pose_rdte_cmd = pose_fb;
  vel_fb << 0, 0, 0, 0, 0, 0;

  RUT::Vector6d wrench_fb_ur, wrench_WTr; // current wrench feedback and transformed wrench world to tool frame
  RUT::Matrix6d stiffness;

  // TODO: use base pointer robot_ptr instead of URRTDE
  //       Need to create interfaces for all used functions here in RobotInterfaces
  URRTDE* urrtde_ptr;

  // Get URRTDE pointer and get initial pose
  if (!_config.mock_hardware) {
    urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
    urrtde_ptr->getCartesian(pose_fb);
  }

  // set initial values for force control
  force_control_ref_pose = pose_fb;
  wrench_WTr.setZero();

  // Initialize control flags
  bool ctrl_flag_saving = false;  // local copy

  //A controller that interpolates linearly between two Cartesian targets.
  // initializes with the current pose and timestamp
  RUT::TaskSpaceInterpolationController intp_controller;
  intp_controller.initialize(pose_fb, timer.toc_ms());
  std::cout << header << "intp_controller initialized with pose_fb: "
            << pose_fb.transpose() << std::endl;

  // this part sets the robot thread state to ready (initialize in manip server)
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_robot_thread_ready[id] = true;
  }

  // profile measures the loop execution time and performance (important for real time control)
  RUT::Profiler loop_profiler;
  std::cout << header << "Loop started." << std::endl;

  RUT::Timer mock_loop_timer;
  mock_loop_timer.set_loop_rate_hz(500);
  mock_loop_timer.start_timed_loop();
  while (true) {
    // Update robot status
    loop_profiler.start();
    RUT::TimePoint t_start;
    double time_now_ms;
    if (!_config.mock_hardware) {
      // updates robot states 
      // UR uses rtde_init_period() + rtde_wait_period() to enforce a 2 ms period. FRANKA???? NI IDEA
      t_start = urrtde_ptr->rtde_init_period();
      urrtde_ptr->getCartesian(pose_fb);
      urrtde_ptr->getCartesianVelocity(vel_fb);
      urrtde_ptr->getWrenchTool(wrench_fb_ur);
      time_now_ms = timer.toc_ms();

      // save feedback in buffers
      loop_profiler.stop("compute");
      loop_profiler.start();
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        _poses_fb[id] = pose_fb;
      }
      loop_profiler.stop("lock");
      loop_profiler.start();

    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      pose_fb = pose_rdte_cmd;
      vel_fb.setZero();
      wrench_fb_ur.setZero();
    }

    // buffer robot pose, velocity and wrench (save data)
    loop_profiler.stop("compute");
    loop_profiler.start();
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      _pose_buffers[id].put(pose_fb);
      _pose_timestamp_ms_buffers[id].put(time_now_ms);
    }
    {
      std::lock_guard<std::mutex> lock(_vel_buffer_mtxs[id]);
      _vel_buffers[id].put(vel_fb);
      _vel_timestamp_ms_buffers[id].put(time_now_ms);
    }
    {
      std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
      _robot_wrench_buffers[id].put(wrench_fb_ur);
      _robot_wrench_timestamp_ms_buffers[id].put(time_now_ms);
    }
    loop_profiler.stop("lock");
    loop_profiler.start();

    // update  target from interpolation controller
    // get_control returns false if no valid target is found, if so, needs to create a new one
    if (!intp_controller.get_control(time_now_ms, force_control_ref_pose)) {
      bool new_wp_found = false;
      {
        // need to get new waypoint from buffer
        std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
        while (!_waypoints_buffers[id].is_empty()) {
          // keep querying buffer until we get a target that is in the future
          pose_target_waypoint = _waypoints_buffers[id].pop();
          double target_time_ms = _waypoints_timestamp_ms_buffers[id].pop();
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(pose_target_waypoint,
                                           target_time_ms);
            new_wp_found = true;
            break;
          }
        }
      }
      if (!new_wp_found) {
        // std::cout << "[debug] time_now_ms: " << time_now_ms
        //           << ", time now: " << timer.toc_ms()
        //           << ", target_time_ms:" << target_time_ms
        //           << ", pose_target_waypoint: "
        //           << pose_target_waypoint.transpose() << std::endl;
        intp_controller.keep_the_last_target(time_now_ms);
      }
      intp_controller.get_control(time_now_ms, force_control_ref_pose);
    }

    loop_profiler.stop("intp_controller");
    loop_profiler.start();

    // update stiffness matrix from buffer

    // condition:
    //   time_now_ms < time[0], do nothing
    //   time_now_ms >= time[0], look for next
    bool new_stiffness_found = false;
    {
      std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
      if (!_stiffness_buffers[id].is_empty()) {
        double next_available_time_ms = _stiffness_timestamp_ms_buffers[id][0];
        if (time_now_ms > next_available_time_ms) {
          new_stiffness_found = true;
          while ((!_stiffness_timestamp_ms_buffers[id].is_empty()) &&
                 (_stiffness_timestamp_ms_buffers[id][0] < time_now_ms)) {
            stiffness = _stiffness_buffers[id].pop();
            next_available_time_ms = _stiffness_timestamp_ms_buffers[id].pop();
          }
        }
      }
    }
    loop_profiler.stop("stiffness");
    loop_profiler.start();

    wrench_WTr.setZero();

    // std::cout << "[debug] time: " << time_now_ms
    //           << ", wrench_fb_ur: " << wrench_fb_ur.transpose()
    //           << ", wrench_WTr: " << wrench_WTr.transpose() << std::endl;

    // Update the compliance controller
    {
      std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
      loop_profiler.stop("controller_lock");
      loop_profiler.start();
      _admittance_controllers[id].setRobotStatus(pose_fb, wrench_fb_ur);
      // Update robot reference
      _admittance_controllers[id].setRobotReference(force_control_ref_pose, wrench_WTr);

      // Update stiffness matrix
      if (new_stiffness_found) {
        _admittance_controllers[id].setStiffnessMatrix(stiffness);
      }
      loop_profiler.stop("controller_set");
      loop_profiler.start();
      // Compute the control output
      _admittance_controllers[id].step(pose_rdte_cmd);
      loop_profiler.stop("controller_step");
      loop_profiler.start();
    }

    // Send control command to the robot
    if ((!_config.mock_hardware) &&
        (!urrtde_ptr->streamCartesian(pose_rdte_cmd))) {
      std::cout << header << "streamCartesian failed. Ending thread."
                << std::endl;
      std::cout << header << "last pose_fb: " << pose_fb.transpose()
                << std::endl;
      std::cout << header << "last wrench_fb_ur: " << wrench_fb_ur.transpose()
                << std::endl;
      std::cout << header << "last force_control_ref_pose: "
                << force_control_ref_pose.transpose() << std::endl;
      std::cout << header << "last pose_rdte_cmd: " << pose_rdte_cmd.transpose()
                << std::endl;
      break;
    }

    // std::cout << "t = " << timer.toc_ms()
    //           << ", pose_rdte_cmd: " << pose_rdte_cmd.transpose() << std::endl;

    // logging
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << "[robot thread] Start saving low dim data." << std::endl;
        json_file_start(_ctrl_robot_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_robot_thread_saving[id] = true;
      save_robot_data_json(_ctrl_robot_data_streams[id],
                           _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                           false);
      json_frame_ending(_ctrl_robot_data_streams[id]);
      _states_robot_seq_id[id]++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << "[robot thread] Stop saving low dim data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_robot_data_json(_ctrl_robot_data_streams[id],
                             _states_robot_seq_id[id], timer.toc_ms(), pose_fb,
                             false);
        json_file_ending(_ctrl_robot_data_streams[id]);
        _ctrl_robot_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_robot_thread_saving[id] = false;
      }
    }

    loop_profiler.stop("logging");
    loop_profiler.start();

    // loop control
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[robot thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }

    loop_profiler.stop("lock");

    // loop timing and overrun check
    if (_config.mock_hardware) {
      mock_loop_timer.sleep_till_next();
    } else {
      double overrun_ms = mock_loop_timer.check_for_overrun_ms(false);
      if (overrun_ms > 0) {
        std::cout << "\033[33m";  // set color to bold yellow
        std::cout << header << "Overrun: " << overrun_ms << "ms" << std::endl;
        std::cout << "\033[0m";  // reset color to default
        loop_profiler.show();
      }
      urrtde_ptr->rtde_wait_period(t_start);
      mock_loop_timer.check_for_overrun_ms(
          false);  // just call it to reset the timer
    }
    loop_profiler.clear();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[robot thread] Joined." << std::endl;
}

void ManipServer::eoat_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][EoAT thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  RUT::VectorXd pos_fb = RUT::VectorXd::Zero(1);
  RUT::Vector2d eoat_target_waypoint;
  RUT::Vector2d eoat_cmd;
  if (!_config.mock_hardware) {
    eoat_ptrs[id]->getJoints(pos_fb);
  }
  eoat_cmd << pos_fb, 0;

  bool ctrl_flag_saving = false;  // local copy

  RUT::TaskSpaceInterpolationController intp_controller;
  intp_controller.initialize(eoat_cmd, timer.toc_ms());
  std::cout << header
            << "intp_controller initialized with pos_fb: " << pos_fb.transpose()
            << std::endl;

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_eoat_thread_ready[id] = true;
  }

  std::cout << header << "Loop started." << std::endl;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(100);
  loop_timer.start_timed_loop();
  while (true) {
    // Update EoAT status (for query and logging)
    double time_now_ms;
    if (!_config.mock_hardware) {
      // real hardware
      eoat_ptrs[id]->getJoints(pos_fb);
      time_now_ms = timer.toc_ms();
    } else {
      // mock hardware
      time_now_ms = timer.toc_ms();
      pos_fb[0] = eoat_cmd[0];
    }
    // save state to eoat fb buffer
    {
      std::lock_guard<std::mutex> lock(_eoat_buffer_mtxs[id]);
      _eoat_buffers[id].put(pos_fb);
      _eoat_timestamp_ms_buffers[id].put(time_now_ms);
    }

    // update control target from interpolation controller
    if (!intp_controller.get_control(time_now_ms, eoat_cmd)) {
      bool new_wp_found = false;
      {
        // need to get new waypoint from buffer
        std::lock_guard<std::mutex> lock(_eoat_waypoints_buffer_mtxs[id]);
        while (!_eoat_waypoints_buffers[id].is_empty()) {
          // keep querying buffer until we get a target that is in the future
          eoat_target_waypoint = _eoat_waypoints_buffers[id].pop();
          double target_time_ms =
              _eoat_waypoints_timestamp_ms_buffers[id].pop();
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(eoat_target_waypoint,
                                           target_time_ms);
            new_wp_found = true;
            break;
          }
        }
      }
      if (!new_wp_found) {
        // std::cout << "[debug] time_now_ms: " << time_now_ms
        //           << ", time now: " << timer.toc_ms()
        //           << ", target_time_ms:" << target_time_ms
        //           << ", eoat_target_waypoint: "
        //           << eoat_target_waypoint.transpose() << std::endl;
        intp_controller.keep_the_last_target(time_now_ms);
      }
      intp_controller.get_control(time_now_ms, eoat_cmd);
    }

    // Send command to EoAT
    double force_fb = 0;
    {
      std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
      // TODO: currently, assuming the grasping force is captured by Z axis of the first wrench sensor.
      // Need to find a better way to specify it.
      force_fb = _wrench_fb[id][2];
    }
    eoat_cmd[1] -= force_fb;
    if ((!_config.mock_hardware) && (!eoat_ptrs[id]->setJointsPosForce(
                                        eoat_cmd.head(1), eoat_cmd.tail(1)))) {
      std::cout << header << "setJointsPosForce failed. Ending thread."
                << std::endl;
      std::cout << header << "last pos_fb: " << pos_fb.transpose() << std::endl;
      std::cout << header << "last eoat_cmd: " << eoat_cmd.transpose()
                << std::endl;
      break;
    }

    // std::cout << "t = " << timer.toc_ms()
    //           << ", eoat_cmd: " << eoat_cmd.transpose() << std::endl;

    // logging
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << header << "Start saving eoat data." << std::endl;
        json_file_start(_ctrl_eoat_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_eoat_thread_saving[id] = true;
      save_eoat_data_json(_ctrl_eoat_data_streams[id], _states_eoat_seq_id[id],
                          timer.toc_ms(), pos_fb);
      json_frame_ending(_ctrl_eoat_data_streams[id]);
      _states_eoat_seq_id[id]++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << header << "Stop saving eoat data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_eoat_data_json(_ctrl_eoat_data_streams[id],
                            _states_eoat_seq_id[id], timer.toc_ms(), pos_fb);
        json_file_ending(_ctrl_eoat_data_streams[id]);
        _ctrl_eoat_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_eoat_thread_saving[id] = false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header
                  << "_ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }

    loop_timer.sleep_till_next();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[EoAT thread] Joined." << std::endl;
}

void ManipServer::joint_sensor_wrench_loop(const RUT::TimePoint& time0, int publish_rate, int id) {
  std::string header = "[ManipServer][Robot Wrench thread] " + std::to_string(id) + ": ";
  std::cout << header << "thread starting." << std::endl;
  
  RUT::Timer timer;
  timer.tic(time0);

  RUT::Vector6d wrench_fb;

  // wait for robot wrench buffer to be populated
  std::cout << header << "Waiting for robot thread to populate robot wrench buffer.\n";
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
      std::cout << header << "Checking buffer[" << id << "], size: " << _robot_wrench_buffers[id].size() << std::endl;
      if (_robot_wrench_buffers[id].size() > 0) {
        std::cout << header << "Buffer populated! Starting main loop." << std::endl;
        break;
      }
    }
    usleep(300 * 1000);  // 300ms
  }

  // Set up timing - can be faster than external sensors since data is already collected
  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(publish_rate);  // Match robot frequency
  loop_timer.start_timed_loop();

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_wrench_thread_ready[id] = true;
  }
  std::cout << header << "Thread marked as ready." << std::endl;

  std::cout << header << "Loop started." << std::endl;
  bool ctrl_flag_saving = false;

  while (true) {
    double time_now_ms;
    
    if (!_config.mock_hardware) {
      // Read from buffer (populated by robot_impedance_loop at 1kHz)
      {
        std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
        if (!_robot_wrench_buffers[id].is_empty()) {
          wrench_fb = _robot_wrench_buffers[id].pop();  
        }
      }
      time_now_ms = timer.toc_ms();     
      // Update external wrench buffers (for compatibility with other threads)
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
      {
        std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
        _wrench_fb[id] = wrench_fb;
      }
    } else {
      // Mock hardware
      wrench_fb.setZero();
      time_now_ms = timer.toc_ms();
    }

    // JSON logging (same as ext_sensor_wrench_loop)
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << "[robot wrench thread] Start saving wrench data." << std::endl;
        json_file_start(_ctrl_wrench_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_wrench_thread_saving[id] = true;
      save_wrench_data_json(_ctrl_wrench_data_streams[id],
                           _states_wrench_seq_id[id], timer.toc_ms(),
                           wrench_fb);
      json_frame_ending(_ctrl_wrench_data_streams[id]);
      _states_wrench_seq_id[id]++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << "[robot wrench thread] Stop saving wrench data." << std::endl;
        save_wrench_data_json(_ctrl_wrench_data_streams[id],
                             _states_wrench_seq_id[id], timer.toc_ms(),
                             wrench_fb);
        json_file_ending(_ctrl_wrench_data_streams[id]);
        _ctrl_wrench_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_wrench_thread_saving[id] = false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[robot wrench thread] _ctrl_flag_running is false. Shutting down this thread." << std::endl;
        break;
      }
    }

    loop_timer.sleep_till_next();
  }

  std::cout << "[robot wrench thread] Joined." << std::endl;
}

//--------WRENCH READING LOOP FROM EXTERNAL SENSOR ---------//
void ManipServer::ext_sensor_wrench_loop(const RUT::TimePoint& time0, int publish_rate,
                              int id) {
  std::string header =
      "[ManipServer][Wrench thread] " + std::to_string(id) + ": ";
  std::cout << header << "thread starting." << std::endl;
  std::cout << header << "Rate at" << publish_rate << "Hz." << std::endl;
  RUT::Timer timer;
  timer.tic(time0);  // so this timer is synced with the main timer

  int num_ft_sensors = force_sensor_ptrs[id]->getNumSensors();
  RUT::VectorXd wrench_fb;

  if (!_config.mock_hardware) {
    // wait for force sensor to be ready
    std::cout << header
              << "Waiting for force sensor to start "
                 "streaming.\n";
    while (!force_sensor_ptrs[id]->is_data_ready()) {
      usleep(100000);
    }
  }

  // wait for pose_fb to be ready
  std::cout << header
            << "Waiting for robot thread to "
               "populate pose_fb. \n";
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      if (_pose_buffers[id].size() > 0) {
        break;
      }
    }
    usleep(300 * 1000);  // 300ms
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_wrench_thread_ready[id] = true;
  }

  std::cout << header << "Loop started." << std::endl;
  bool ctrl_flag_saving = false;  // local copy

  RUT::Vector7d pose_fb;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(publish_rate);
  loop_timer.start_timed_loop();
  while (true) {
    // Update robot status
    RUT::TimePoint t_start;
    double time_now_ms;
    if (!_config.mock_hardware) {
      // get the most recent tool pose (for static calibration)
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        pose_fb = _poses_fb[id];
      }
      int safety_flag =
          force_sensor_ptrs[id]->getWrenchNetTool(pose_fb, wrench_fb);
      if (safety_flag < 0) {
        std::cout << header
                  << "Wrench is above safety threshold. Ending thread."
                  << std::endl;
        break;
      }
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
      {
        std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
        _wrench_fb[id] = wrench_fb;
      }
    } else {
      // mock hardware
      wrench_fb.setZero(num_ft_sensors * 6);
      time_now_ms = timer.toc_ms();
      {
        std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
        _wrench_buffers[id].put(wrench_fb);
        _wrench_timestamp_ms_buffers[id].put(time_now_ms);
      }
    }

    // logging
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << "[wrench thread] Start saving wrench data." << std::endl;
        json_file_start(_ctrl_wrench_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_wrench_thread_saving[id] = true;
      save_wrench_data_json(_ctrl_wrench_data_streams[id],
                            _states_wrench_seq_id[id], timer.toc_ms(),
                            wrench_fb);
      json_frame_ending(_ctrl_wrench_data_streams[id]);
      _states_wrench_seq_id[id]++;
    } else {
      _ctrl_mtx.unlock();

      if (ctrl_flag_saving) {
        std::cout << "[wrench thread] Stop saving wrench data." << std::endl;
        // save one last frame, so we can do the correct different frame ending
        save_wrench_data_json(_ctrl_wrench_data_streams[id],
                              _states_wrench_seq_id[id], timer.toc_ms(),
                              wrench_fb);
        json_file_ending(_ctrl_wrench_data_streams[id]);
        _ctrl_wrench_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_wrench_thread_saving[id] = false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[wrench thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;
        break;
      }
    }
    loop_timer.sleep_till_next();
  }  // end of while loop

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[wrench thread] Joined." << std::endl;
}

void ManipServer::rgb_loop(const RUT::TimePoint& time0, int id) {
  std::string header = "[ManipServer][rgb thread] " + std::to_string(id) + ": ";
  std::cout << header << "starting thread" << std::endl;

  RUT::Timer timer;
  timer.tic(time0);
  double time_start = timer.toc_ms();
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_rgb_thread_ready[id] = true;
  }
  std::cout << header << "Loop started." << std::endl;

  cv::Mat resized_color_mat;
  cv::Mat bgr[3];  //destination array
  Eigen::MatrixXd bm, gm, rm;
  Eigen::MatrixXd rgb_row_combined;

  while (true) {
    double time_now_ms = 0;
    {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      if (!_config.mock_hardware) {
        _color_mats[id] = camera_ptrs[id]->next_rgb_frame_blocking();
      } else {
        // mock hardware
        _color_mats[id] = cv::Mat::zeros(1080, 1080, CV_8UC3);
        usleep(20 * 1000);  // 20ms, 50hz
      }
      time_now_ms = timer.toc_ms();
      cv::resize(_color_mats[id], resized_color_mat,
                 cv::Size(_config.output_rgb_hw[1], _config.output_rgb_hw[0]),
                 cv::INTER_LINEAR);
      cv::split(resized_color_mat, bgr);  //split source
    }

    cv::cv2eigen(bgr[0], bm);
    cv::cv2eigen(bgr[1], gm);
    cv::cv2eigen(bgr[2], rm);
    rgb_row_combined.resize(resized_color_mat.rows * 3, resized_color_mat.cols);
    rgb_row_combined << rm, gm, bm;
    {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
      _camera_rgb_buffers[id].put(rgb_row_combined);
      _camera_rgb_timestamp_ms_buffers[id].put(time_now_ms);
    }

    if (_ctrl_flag_saving) {
      _states_rgb_thread_saving[id] = true;
      {
        std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
        save_rgb_data(_ctrl_rgb_folders[id], _states_rgb_seq_id[id],
                      timer.toc_ms(), _color_mats[id]);
      }
      _states_rgb_seq_id[id]++;
    } else {
      _states_rgb_thread_saving[id] = false;
    }

    // std::cout << "t = " << timer.toc_ms() << ", get new rgb frame."
    //           << std::endl;
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << "[rgb thread] _ctrl_flag_running is false. Shuting "
                     "down this thread."
                  << std::endl;

        break;
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  std::cout << "[rgb thread] Joined." << std::endl;
}

void ManipServer::rgb_plot_loop() {
  std::string header = "[ManipServer][plot thread]: ";
  std::cout << header << "starting thread." << std::endl;
  cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
  std::vector<cv::Mat> color_mat_copy;
  cv::Mat canvas;

  for (int id : _id_list) {
    color_mat_copy.push_back(cv::Mat());
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _state_plot_thread_ready = true;
  }

  std::cout << header << "Loop started." << std::endl;

  while (true) {
    for (int id : _id_list) {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      color_mat_copy[id] = _color_mats[id].clone();
    }

    cv::vconcat(color_mat_copy, canvas);

    cv::imshow("RGB", canvas);

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header
                  << "[rgb plot thread] _ctrl_flag_running is false. Shuting "
                     "down this thread"
                  << std::endl;
        break;
      }
    }

    if (cv::waitKey(30) >= 0)
      break;
  }
  std::cout << "[plot thread] Joined." << std::endl;
}

void ManipServer::robot_logging_loop(const RUT::TimePoint& time0, int id) {
  std::string header = "[ManipServer][Robot Logging thread] " + std::to_string(id) + ": ";
  std::cout << header << "starting thread" << std::endl;

  RUT::Timer timer;
  timer.tic(time0);
  
  bool ctrl_flag_saving = false;
  int frames_written = 0;
  
  // Process buffer at 200Hz (fast enough to handle 1kHz data generation)
  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(200);
  loop_timer.start_timed_loop();

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_logging_thread_ready[id] = true;
  }
  std::cout << header << "Loop started." << std::endl;

  while (true) {
    _ctrl_mtx.lock();
    bool should_save = _ctrl_flag_saving;
    _ctrl_mtx.unlock();

    if (should_save) {
      if (!ctrl_flag_saving) {
        std::cout << header << "Start saving low dim data." << std::endl;
        json_file_start(_ctrl_robot_data_streams[id]);
        json_file_start(_ctrl_torque_data_streams[id]);
        json_file_start(_ctrl_joint_data_streams[id]);
        ctrl_flag_saving = true;
        frames_written = 0;
      }

      // Batch process up to 100 frames per iteration (5ms worth at 1kHz)
      int batch_count = 0;
      while (batch_count < 100) {
        RobotLogData log_data;
        bool has_data = false;
        
        {
          std::lock_guard<std::mutex> lock(_logging_buffer_mtxs[id]);
          if (!_logging_buffers[id].is_empty()) {
            log_data = _logging_buffers[id].pop();
            has_data = true;
          }
        }
        
        if (!has_data) break;
        
        // Write to JSON with torques
        save_robot_data_json(_ctrl_robot_data_streams[id],
                           _states_robot_seq_id[id], 
                           log_data.timestamp_ms, 
                           log_data.pose_fb,
                           false);

        // Write torque-only JSON
        save_robot_torque_json(_ctrl_torque_data_streams[id],
                               _states_robot_seq_id[id],
                               log_data.timestamp_ms,
                               log_data.tau_J,
                               false);
        
        // Write joint positions JSON
        save_robot_joint_positions_json(_ctrl_joint_data_streams[id],
                                        _states_robot_seq_id[id],
                                        log_data.timestamp_ms,
                                        log_data.q,
                                        false);
        
        json_frame_ending(_ctrl_robot_data_streams[id]);
        json_frame_ending(_ctrl_torque_data_streams[id]);
        json_frame_ending(_ctrl_joint_data_streams[id]);
        _states_robot_seq_id[id]++;
        frames_written++;
        batch_count++;
      }
      
      // Check for overflow warning
      if (_logging_buffer_overflow[id].load()) {
        std::cerr << header << "Buffer overflow - data loss may occur!" << std::endl;
      }
      
    } else {
      if (ctrl_flag_saving) {
        std::cout << header << "Stop saving low dim data. Flushing buffer..." << std::endl;
        
        // Flush all remaining data
        int flushed = 0;
        RobotLogData last_log_data;
        bool has_last_data = false;
        
        while (true) {
          RobotLogData log_data;
          bool has_data = false;
          
          {
            std::lock_guard<std::mutex> lock(_logging_buffer_mtxs[id]);
            if (!_logging_buffers[id].is_empty()) {
              log_data = _logging_buffers[id].pop();
              has_data = true;
            }
          }
          
          if (!has_data) break;
          
          // If we have a previous frame, write it with frame ending
          if (has_last_data) {
            save_robot_data_json(_ctrl_robot_data_streams[id],
                               _states_robot_seq_id[id],
                               last_log_data.timestamp_ms,
                               last_log_data.pose_fb,
                               false);
            json_frame_ending(_ctrl_robot_data_streams[id]);
            save_robot_torque_json(_ctrl_torque_data_streams[id],
                                   _states_robot_seq_id[id],
                                   last_log_data.timestamp_ms,
                                   last_log_data.tau_J,
                                   false);
            json_frame_ending(_ctrl_torque_data_streams[id]);
            save_robot_joint_positions_json(_ctrl_joint_data_streams[id],
                                            _states_robot_seq_id[id],
                                            last_log_data.timestamp_ms,
                                            last_log_data.q,
                                            false);
            json_frame_ending(_ctrl_joint_data_streams[id]);
            _states_robot_seq_id[id]++;
            flushed++;
          }
          
          // Store current as last
          last_log_data = log_data;
          has_last_data = true;
        }
        
        // Write the very last frame without json_frame_ending
        if (has_last_data) {
          save_robot_data_json(_ctrl_robot_data_streams[id],
                             _states_robot_seq_id[id],
                             last_log_data.timestamp_ms,
                             last_log_data.pose_fb,
                             false);
          _states_robot_seq_id[id]++;
          flushed++;
          save_robot_torque_json(_ctrl_torque_data_streams[id],
                                 _states_robot_seq_id[id]-1,
                                 last_log_data.timestamp_ms,
                                 last_log_data.tau_J,
                                 false);
          save_robot_joint_positions_json(_ctrl_joint_data_streams[id],
                                          _states_robot_seq_id[id]-1,
                                          last_log_data.timestamp_ms,
                                          last_log_data.q,
                                          false);
        }
        
        std::cout << header << "Flushed " << flushed << " remaining frames." << std::endl;
        std::cout << header << "Total frames written: " << frames_written + flushed << std::endl;
        
        json_file_ending(_ctrl_robot_data_streams[id]);
        _ctrl_robot_data_streams[id].close();
        json_file_ending(_ctrl_torque_data_streams[id]);
        _ctrl_torque_data_streams[id].close();
        json_file_ending(_ctrl_joint_data_streams[id]);
        _ctrl_joint_data_streams[id].close();
        ctrl_flag_saving = false;
        _states_robot_thread_saving[id] = false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header << "_ctrl_flag_running is false. Shutting down." << std::endl;
        break;
      }
    }

    loop_timer.sleep_till_next();
  }
  
  
  std::cout << header << "Joined." << std::endl;
}

// Teleoperation control loop for Gamepad input
void ManipServer::teleop_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][Teleop thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  RUT::Timer timer;
  timer.tic(time0);

  // Verify Gamepad pointer exists
  if (!gamepad_ptrs[id]) {
    std::cout << header << "Gamepad pointer is null. Exiting.\n";
    return;
  }
  std::cout << header << "Using Gamepad for teleoperation.\n";

  // Wait for robot feedback to be available
  std::cout << header << "Waiting for robot feedback to be available...\n";
  RUT::Vector7d current_pose = RUT::Vector7d::Zero();
  int retry_count = 0;
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
      if (!_poses_fb[id].isZero()) {
        current_pose = _poses_fb[id];
        std::cout << header << "Got initial pose: " << current_pose.transpose() << "\n";
        break;
      }
    }
    retry_count++;
    if (retry_count > 100) {
      std::cerr << header << "Timeout waiting for robot feedback. Exiting.\n";
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  RUT::Vector7d target_pose = current_pose;
  GamepadData gp_data;

  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(200);  // 200Hz teleoperation loop
  loop_timer.start_timed_loop();

  // Scaling factors from YAML
  double TRANSLATION_SCALE = 1;  // fallback default
  double ROTATION_SCALE = 1;     // fallback default
  if (_teleop_translation_scales.size() > id) {
    TRANSLATION_SCALE = _teleop_translation_scales[id];
  }
  if (_teleop_rotation_scales.size() > id) {
    ROTATION_SCALE = _teleop_rotation_scales[id];
  }

  // refresh base pose
  RUT::Vector7d base_pose;
  {
    std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
    base_pose = _poses_fb[id];
  }

  std::cout << header << "Gamepad teleoperation active (left stick = translation, right stick = rotation, LT/LB = translation in Z)\n";

  while (true) {
    // Check if we should exit
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        break;
      }
    }

    double tx_scaled = 0, ty_scaled = 0, tz_scaled = 0;
    double rx_scaled = 0, ry_scaled = 0, rz_scaled = 0;

    // Read Gamepad data
    if (gamepad_ptrs[id]->get_data(gp_data)) {
      // Left stick to translation (X, Y, Z from sticks and triggers)
      tx_scaled = gp_data.left_stick_y * TRANSLATION_SCALE;
      ty_scaled = -gp_data.left_stick_x * TRANSLATION_SCALE;
      tz_scaled = (gp_data.left_trigger - gp_data.right_trigger) * TRANSLATION_SCALE;
      
      // Right stick to rotation
      rx_scaled = gp_data.right_stick_x * ROTATION_SCALE;
      ry_scaled = gp_data.right_stick_y * ROTATION_SCALE;
      rz_scaled = 0.0;
    }

    // Check for significant movement
    double tx_norm = std::abs(tx_scaled);
    double ty_norm = std::abs(ty_scaled);
    double tz_norm = std::abs(tz_scaled);
    double angle = std::sqrt(rx_scaled * rx_scaled + ry_scaled * ry_scaled + rz_scaled * rz_scaled);
    
    // Only apply incremental update if movement is significant
    if (tx_norm > 1e-9 || ty_norm > 1e-9 || tz_norm > 1e-9 || angle > 1e-6) {
      
      // Apply translation
      target_pose(0) += tx_scaled;
      target_pose(1) += ty_scaled;
      target_pose(2) += tz_scaled;

      // Apply rotation
      /*
      if (angle > 1e-6) {
        Eigen::Vector3d axis(rx_scaled, ry_scaled, rz_scaled);
        if (axis.norm() > 1e-12) {
          axis.normalize();
          Eigen::Quaterniond rot_incr(Eigen::AngleAxisd(angle, axis));
          Eigen::Quaterniond base_q(base_pose(6), base_pose(3), base_pose(4), base_pose(5));
          Eigen::Quaterniond new_q = rot_incr * base_q;
          new_q.normalize();
          target_pose(3) = new_q.x();
          target_pose(4) = new_q.y();
          target_pose(5) = new_q.z();
          target_pose(6) = new_q.w();
        } else {
          target_pose(3) = base_pose(3);
          target_pose(4) = base_pose(4);
          target_pose(5) = base_pose(5);
          target_pose(6) = base_pose(6);
        }
      } else {
        target_pose(3) = base_pose(3);
        target_pose(4) = base_pose(4);
        target_pose(5) = base_pose(5);
        target_pose(6) = base_pose(6);
      }*/
    } else {
      // keep last target if no significant movement
      target_pose = target_pose;
    }

    // Schedule waypoint for the robot
    set_target_pose(target_pose, 1, id);  // 100ms dt

    loop_timer.sleep_till_next();
  }
}

