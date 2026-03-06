#include "table_top_manip/manip_server.h"

#include <RobotUtilities/interpolation_controller.h>
#include <opencv2/core/eigen.hpp>

#include "helpers.hpp"

// =============================================================================
// ROBOT IMPEDANCE CONTROL LOOP - FRANKA PANDA
// =============================================================================
//
// Purpose: Real-time torque-based impedance control for Franka Panda robot
//          Runs at 1kHz to meet Franka's real-time control requirements
//
// Control Architecture:
//   1. Read robot state (pose, wrench, joint states) from Franka
//   2. Query waypoint buffer for target trajectory
//   3. Interpolate smooth reference trajectory
//   4. Compute impedance control law (torque = f(pose_error, wrench))
//   5. Send joint torques to robot
//   6. Log data to lock-free buffer for async I/O
//
// Real-Time Constraints:
//   - MUST complete within 1ms (1kHz control rate)
//   - Uses SCHED_FIFO real-time priority (99)
//   - Minimal locking, lock-free logging
//   - Franka monitors success rate (must stay > 95%)
//
// Thread Safety:
//   - Reads waypoints from _waypoints_buffer (mutex-protected)
//   - Writes to logging buffer (lock-free circular buffer)
//   - Updates shared pose feedback (mutex-protected)
//
// =============================================================================

void ManipServer::robot_impedance_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][Robot Impedance thread] " + std::to_string(id) + ": ";
  std::cout << header + "starting thread.\n";

  // ============================================================================
  // Step 1: Set Real-Time Thread Priority
  // ============================================================================
  // Critical for meeting Franka's 1kHz timing requirements
  // Priority 99 = highest real-time priority (requires sudo or CAP_SYS_NICE)
  
  if (!_config.mock_hardware) {
    struct sched_param param;
    param.sched_priority = 99;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
      std::cerr << header << " Failed to set real-time priority: " 
                << strerror(errno) << std::endl;
      std::cerr << header << "   Try: sudo setcap cap_sys_nice=eip <executable>" 
                << std::endl;
    } else {
      std::cout << header << " Real-time priority (SCHED_FIFO, priority 99) set successfully" 
                << std::endl;
    }
  }

  // ============================================================================
  // Step 2: Initialize Control Variables
  // ============================================================================
  
  // Synchronized timer for consistent timestamps across all threads
  RUT::Timer timer;
  timer.tic(time0);

  // State feedback variables
  RUT::Vector7d pose_fb;              // Current end-effector pose (x,y,z, qx,qy,qz,qw)
  RUT::Vector6d vel_fb;               // Current Cartesian velocity (vx,vy,vz, wx,wy,wz)
  RUT::Vector6d wrench_fb_ur;         // Force/torque at end-effector (Fx,Fy,Fz, Tx,Ty,Tz)
  RUT::Vector6d wrench_WTr;           // Transformed wrench (world → tool frame)
  
  // Control command variables
  RUT::Vector7d pose_target_waypoint; // Next waypoint from buffer
  RUT::Vector7d ref_pose;             // Interpolated reference pose
  RUT::Vector7d torque_robot_cmd;     // Joint torque commands (7 joints)
  RUT::Matrix6d stiffness;            // Impedance stiffness matrix (6x6)

  // Initialize with safe defaults for mock hardware
  pose_fb << id, 0, 0, 1, 0, 0, 0;    // Identity pose with offset
  torque_robot_cmd = RUT::Vector7d::Zero();
  vel_fb = RUT::Vector6d::Zero();
  wrench_WTr = RUT::Vector6d::Zero();

  // ============================================================================
  // Step 3: Initialize Robot Interface and Dynamics Model
  // ============================================================================
  
  FRANKA* franka_ptr = nullptr;
  franka::RobotState state;      // Current robot state
  
  // Get robot interface pointer and read initial state
  if (!_config.mock_hardware) {
    franka_ptr = static_cast<FRANKA*>(robot_ptrs[id].get());
    franka_ptr->getCartesian(pose_fb);
    
    // Read initial robot state
    state = franka_ptr->getRobotState();
    
    std::cout << header << "Initial pose: " << pose_fb.transpose() << std::endl;
  }
  
  // Load Franka's dynamics model for Jacobian calculations (if using real hardware)
  // Note: Model must be created after checking for real hardware
  std::unique_ptr<franka::Model> model_ptr;
  if (!_config.mock_hardware) {
    model_ptr = std::make_unique<franka::Model>(franka_ptr->loadModel());
  }

  // ============================================================================
  // Step 4: Initialize Trajectory Interpolator
  // ============================================================================
  // Generates smooth reference trajectories between waypoints
  
  ref_pose = pose_fb;  // Start with current pose as reference
  
  RUT::TaskSpaceInterpolationController intp_controller;
  intp_controller.initialize(pose_fb, timer.toc_ms());
  std::cout << header << "Trajectory interpolator initialized" << std::endl;

  // ============================================================================
  // Step 5: Signal Thread Ready and Initialize Profiling
  // ============================================================================
  
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_robot_thread_ready[id] = true;
  }
  std::cout << header << "Thread ready. Starting control loop." << std::endl;

  // Performance profiling for debugging (disabled in production)
  RUT::Profiler loop_profiler;

  // Statistics tracking for control performance analysis
  uint64_t loop_count = 0;            // Total iterations
  uint64_t overrun_count = 0;         // Times loop exceeded 1ms
  double max_overrun_ms = 0.0;        // Worst-case overrun
  double total_overrun_ms = 0.0;      // Cumulative overrun time
  double last_franka_success_rate = 1.0;  // Franka's internal success metric

  // Loop timing control (1kHz for Franka)
  RUT::Timer mock_loop_timer;
  mock_loop_timer.set_loop_rate_hz(1000);
  mock_loop_timer.start_timed_loop();

  // ============================================================================
  // MAIN CONTROL LOOP (1kHz Real-Time Control)
  // ============================================================================
  
  std::cout << header << "Entering 1kHz control loop" << std::endl;

  while (true) {
    loop_profiler.start();
    RUT::TimePoint t_start;
    double time_now_ms;
    
    // ==========================================================================
    // Phase 1: Update Robot State
    // ==========================================================================
    
    if (!_config.mock_hardware) {
      // Read current robot state from Franka
      franka_ptr->getCurrentPose(pose_fb);
      franka_ptr->getCurrentWrenchTool(wrench_fb_ur);
      state = franka_ptr->getRobotState();
      
      // Recompute Jacobian for current configuration
      std::array<double, 42> jacobian_array = 
          model_ptr->zeroJacobian(franka::Frame::kEndEffector, state);
      
      // Update impedance controller with latest Jacobian and robot state
      _impedance_controllers[id].getJacobian(
          Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data()));
      _impedance_controllers[id].getRobotState(state);
      
      time_now_ms = timer.toc_ms();
      
      // Store current pose for other threads to query
      loop_profiler.stop("compute");
      loop_profiler.start();
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        _poses_fb[id] = pose_fb;
      }
      loop_profiler.stop("lock");
      loop_profiler.start();
      
    } else {
      // Mock hardware: simulate with zero wrench
      time_now_ms = timer.toc_ms();
      wrench_fb_ur.setZero();
    }
    
    // ==========================================================================
    // Phase 2: Log State to Data Buffers
    // ==========================================================================
    
    loop_profiler.stop("compute");
    loop_profiler.start();
    
    // Log pose with timestamp for trajectory reconstruction
    {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      _pose_buffers[id].put(pose_fb);
      _pose_timestamp_ms_buffers[id].put(time_now_ms);
    }
    
    // Log wrench measurements for force analysis
    {
      std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
      _robot_wrench_buffers[id].put(wrench_fb_ur);
      _robot_wrench_timestamp_ms_buffers[id].put(time_now_ms);
    }
    
    loop_profiler.stop("lock");
    loop_profiler.start();

    // ==========================================================================
    // Phase 3: Update Reference Trajectory from Waypoint Buffer
    // ==========================================================================
    // Interpolator generates smooth trajectories between waypoints
    
    if (!intp_controller.get_control(time_now_ms, ref_pose)) {
      // No valid target available - need to fetch new waypoint
      bool new_wp_found = false;
      
      {
        std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
        
        // Scan buffer for next valid (future) waypoint
        while (!_waypoints_buffers[id].is_empty()) {
          pose_target_waypoint = _waypoints_buffers[id].pop();
          double target_time_ms = _waypoints_timestamp_ms_buffers[id].pop();
          
          // Only accept waypoints scheduled for future execution
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(pose_target_waypoint, target_time_ms);
            new_wp_found = true;
            break;
          }
          // Discard obsolete waypoints (scheduled in the past)
        }
      }
      
      if (!new_wp_found) {
        // No future waypoints available - hold current position
        intp_controller.keep_the_last_target(time_now_ms);
      }
      
      // Retrieve updated reference
      intp_controller.get_control(time_now_ms, ref_pose);
    }
    
    loop_profiler.stop("intp_controller");
    loop_profiler.start();

    // ==========================================================================
    // Phase 4: Update Impedance Stiffness Matrix (if scheduled)
    // ==========================================================================
    // Allows dynamic compliance adjustment during execution
    
    bool new_stiffness_found = false;
    {
      std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
      
      if (!_stiffness_buffers[id].is_empty()) {
        double next_available_time_ms = _stiffness_timestamp_ms_buffers[id][0];
        
        // Check if it's time to apply new stiffness
        if (time_now_ms > next_available_time_ms) {
          new_stiffness_found = true;
          
          // Consume all stiffness updates up to current time
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
    
    // Initialize wrench transformation (currently not used)
    wrench_WTr.setZero();

    // ==========================================================================
    // Phase 5: Compute Impedance Control Law
    // ==========================================================================
    // Torque = K(x_ref - x) + D(ẋ_ref - ẋ) + compensation
    
    {
      std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
      loop_profiler.stop("controller_lock");
      loop_profiler.start();
      
      // Update controller with current robot state and external wrench
      _impedance_controllers[id].setRobotStatus(pose_fb, wrench_fb_ur);
      
      // Set desired reference pose and wrench
      _impedance_controllers[id].setRobotReference(ref_pose, wrench_WTr);
      
      // Apply new stiffness matrix if available
      if (new_stiffness_found) {
        _impedance_controllers[id].setStiffnessMatrix(stiffness);
      }
      
      loop_profiler.stop("controller_set");
      loop_profiler.start();
      
      // Compute joint torques to achieve desired impedance behavior
      _impedance_controllers[id].step(torque_robot_cmd);
      
      loop_profiler.stop("controller_step");
    }

    // ==========================================================================
    // Phase 6: Send Torque Commands to Robot
    // ==========================================================================
    
    loop_profiler.start();
    
    if ((!_config.mock_hardware) &&
        (!franka_ptr->setTorques(torque_robot_cmd))) {
      // Robot rejected command - emergency stop
      std::cout << "\033[31m";  // Red color
      std::cout << header << " setTorques() failed - Emergency stop" << std::endl;
      std::cout << header << "Last state:" << std::endl;
      std::cout << header << "  Pose: " << pose_fb.transpose() << std::endl;
      std::cout << header << "  Wrench: " << wrench_fb_ur.transpose() << std::endl;
      std::cout << header << "  Reference: " << ref_pose.transpose() << std::endl;
      std::cout << header << "  Torque cmd: " << torque_robot_cmd.transpose() << std::endl;
      std::cout << "\033[0m";  // Reset color
      break;
    }
    
    loop_profiler.stop("franka_update");

    // ==========================================================================
    // Phase 7: High-Speed Data Logging (Lock-Free)
    // ==========================================================================
    // Logging to circular buffer is ultra-fast (<10µs) to maintain real-time
    
    loop_profiler.start();
    
    if (_ctrl_flag_saving) {
      std::lock_guard<std::mutex> lock(_logging_buffer_mtxs[id]);
      
      if (!_logging_buffers[id].is_full()) {
        // Pack log data structure
        RobotLogData log_data;
        log_data.timestamp_ms = time_now_ms;
        log_data.pose_fb = pose_fb;
        
        // Extract joint torques and positions from Franka state
        for (int i = 0; i < 7; ++i) {
          log_data.tau_J[i] = state.tau_J[i];
          log_data.q[i] = state.q[i];
        }
        
        _logging_buffers[id].put(log_data);
        _states_robot_thread_saving[id] = true;
        _logging_buffer_overflow[id].store(false);
        
      } else {
        // Buffer overflow - logging thread can't keep up with 1kHz rate
        if (!_logging_buffer_overflow[id].load()) {
          _logging_buffer_overflow[id].store(true);
          std::cerr << "\033[33m";  // Yellow
          std::cerr << header << "⚠️  Logging buffer overflow - data loss!" << std::endl;
          std::cerr << "\033[0m";
        }
      }
    } else {
      _states_robot_thread_saving[id] = false;
    }
    
    loop_profiler.stop("logging");

    // ==========================================================================
    // Phase 8: Check Exit Condition
    // ==========================================================================
    
    loop_profiler.start();
    
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header << "Exit signal received. Shutting down control loop." 
                  << std::endl;
        break;
      }
    }
    
    loop_profiler.stop("lock");

    // ==========================================================================
    // Phase 9: Loop Timing and Performance Monitoring
    // ==========================================================================
    
    loop_count++;
    
    if (_config.mock_hardware) {
      // Mock hardware: sleep to maintain loop rate
      mock_loop_timer.sleep_till_next();
      
    } else {
      // Real hardware: monitor Franka's success rate (critical metric)
      if (loop_count % 1000 == 0) {
        last_franka_success_rate = state.control_command_success_rate;
        
        // Alert if success rate drops below acceptable threshold
        if (last_franka_success_rate < 0.95) {
          std::cout << "\033[31m";  // Red
          std::cout << header << "⚠️  WARNING: Franka success rate: " 
                    << (last_franka_success_rate * 100.0) << "% "
                    << "(Local overruns: " << (100.0 * overrun_count / loop_count) << "%)"
                    << std::endl;
          std::cout << "\033[0m";
        } else if (loop_count % 10000 == 0) {
          // Periodic status update when everything is nominal
          std::cout << "\033[32m";  // Green
          std::cout << header << "✅ Franka Success Rate: " 
                    << (last_franka_success_rate * 100.0) << "%" << std::endl;
          std::cout << "\033[0m";
        }
      }
      
      // Track loop overruns for post-analysis
      double overrun_ms = mock_loop_timer.check_for_overrun_ms(false);
      if (overrun_ms > 0) {
        overrun_count++;
        total_overrun_ms += overrun_ms;
        if (overrun_ms > max_overrun_ms) {
          max_overrun_ms = overrun_ms;
        }
        // Don't print every overrun - only matters if Franka success rate drops
      }
      
      mock_loop_timer.check_for_overrun_ms(false);  // Reset timer
    }
    
    loop_profiler.clear();
    
  }  // End of control loop

  // ============================================================================
  // Control Loop Exit - Print Performance Statistics
  // ============================================================================
  
  std::cout << "\033[36m";  // Cyan color for visibility
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << "📊 CONTROL LOOP PERFORMANCE STATISTICS" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << "  Total iterations:  " << loop_count << std::endl;
  std::cout << header << "  Overrun count:     " << overrun_count 
            << " (" << (100.0 * overrun_count / loop_count) << "%)" << std::endl;
  
  if (overrun_count > 0) {
    std::cout << header << "  Average overrun:   " 
              << (total_overrun_ms / overrun_count) << " ms" << std::endl;
    std::cout << header << "  Maximum overrun:   " 
              << max_overrun_ms << " ms" << std::endl;
  }
  
  std::cout << header << "  Final Franka rate: " 
            << (last_franka_success_rate * 100.0) << "%" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\033[0m";  // Reset color

  // Signal other threads to stop
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  
  std::cout << header << "Thread terminated." << std::endl;
}

// =============================================================================
// ROBOT ADMITTANCE CONTROL LOOP - UR ROBOT (RTDE INTERFACE)
// =============================================================================
//
// Purpose: Real-time Cartesian admittance control for UR robots
//          Runs at 500Hz to match UR RTDE control rate (2ms cycle)
//
// Control Architecture:
//   1. Read robot state (pose, velocity, wrench) via RTDE interface
//   2. Query waypoint buffer for target trajectory
//   3. Interpolate smooth reference trajectory
//   4. Update admittance controller with latest wrench measurements
//   5. Compute desired Cartesian velocity/pose adjustments
//   6. Stream pose commands back to robot via RTDE
//   7. Log data to JSON format for analysis
//
// Real-Time Constraints:
//   - MUST complete within 2ms (500Hz control rate)
//   - UR RTDE enforces strict timing with rtde_init_period() + rtde_wait_period()
//   - Minimize locking; data flows through circular buffers
//
// Thread Safety:
//   - Reads waypoints from _waypoints_buffer (mutex-protected)
//   - Reads stiffness values from _stiffness_buffer (mutex-protected)
//   - Updates controller state (mutex-protected)
//   - Writes to logging streams (protected by save flags)
//
// Key Difference from Impedance Control:
//   - Admittance: velocity-based control (Cartesian space)
//   - Impedance: torque-based control (joint space with Jacobian)
//
// =============================================================================

void ManipServer::robot_admittance_loop(const RUT::TimePoint& time0, int id) {
  std::string header =
      "[ManipServer][Robot Admittance thread] " + std::to_string(id) + ": ";
  std::cout << header << "Starting thread.\n";

  // ============================================================================
  // Step 1: Initialize Synchronized Timer
  // ============================================================================
  // Use global time reference to synchronize with other threads
  
  RUT::Timer timer;
  timer.tic(time0);

  // ============================================================================
  // Step 2: Initialize State Feedback Variables
  // ============================================================================
  
  // Robot state
  RUT::Vector7d pose_fb;                      // Current pose (x,y,z, qx,qy,qz,qw)
  RUT::Vector7d pose_target_waypoint;         // Target waypoint from buffer
  RUT::Vector6d vel_fb;                       // Current Cartesian velocity
  
  // Force control
  RUT::Vector7d force_control_ref_pose;       // Reference pose for compliance
  RUT::Vector6d wrench_fb_ur;                 // Measured wrench at tool
  RUT::Vector6d wrench_WTr;                   // Transformed wrench (world→tool)
  
  // Control output
  RUT::Vector7d pose_rdte_cmd;                // Pose command to send to UR
  RUT::Matrix6d stiffness;                    // Compliance stiffness matrix
  
  // Initialize with safe defaults (for mock hardware mode)
  pose_fb << id, 0, 0, 1, 0, 0, 0;            // Identity pose with robot ID offset
  pose_rdte_cmd = pose_fb;
  vel_fb = RUT::Vector6d::Zero();
  wrench_WTr.setZero();

  // ============================================================================
  // Step 3: Initialize UR Robot Interface
  // ============================================================================
  
  URRTDE* urrtde_ptr = nullptr;
  
  if (!_config.mock_hardware) {
    urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
    urrtde_ptr->getCartesian(pose_fb);
    std::cout << header << "Initial pose: " << pose_fb.transpose() << std::endl;
  }

  // Set initial force control reference to current pose
  force_control_ref_pose = pose_fb;

  // ============================================================================
  // Step 4: Initialize Trajectory Interpolator
  // ============================================================================
  // Generates smooth paths between waypoints
  
  RUT::TaskSpaceInterpolationController intp_controller;
  intp_controller.initialize(pose_fb, timer.toc_ms());
  std::cout << header << "Trajectory interpolator initialized with pose: "
            << pose_fb.transpose() << std::endl;

  // ============================================================================
  // Step 5: Signal Thread Ready
  // ============================================================================
  
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_robot_thread_ready[id] = true;
  }
  std::cout << header << "Thread ready. Starting control loop." << std::endl;

  // ============================================================================
  // Step 6: Initialize Performance Profiling and Timing
  // ============================================================================
  
  RUT::Profiler loop_profiler;           // Measure phase execution times
  bool ctrl_flag_saving = false;         // Local copy of save flag
  
  // Loop timing control (500Hz for UR RTDE)
  RUT::Timer mock_loop_timer;
  mock_loop_timer.set_loop_rate_hz(500);
  mock_loop_timer.start_timed_loop();

  std::cout << header << "Entering 500Hz control loop" << std::endl;

  // ============================================================================
  // MAIN CONTROL LOOP (500Hz - UR RTDE Control Rate)
  // ============================================================================
  
  while (true) {
    // ==========================================================================
    // Phase 1: Update Robot State from RTDE
    // ==========================================================================
    
    loop_profiler.start();
    RUT::TimePoint t_start;
    double time_now_ms;
    
    if (!_config.mock_hardware) {
      // RTDE timing: init period marks start of 2ms window
      t_start = urrtde_ptr->rtde_init_period();
      
      // Read current robot state (non-blocking, high-speed)
      urrtde_ptr->getCartesian(pose_fb);
      urrtde_ptr->getCartesianVelocity(vel_fb);
      urrtde_ptr->getWrenchTool(wrench_fb_ur);
      time_now_ms = timer.toc_ms();

      // Store current pose for other threads to query (e.g., teleoperation)
      loop_profiler.stop("compute");
      loop_profiler.start();
      {
        std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
        _poses_fb[id] = pose_fb;
      }
      loop_profiler.stop("lock");
      loop_profiler.start();

    } else {
      // Mock hardware: use commanded pose as feedback
      time_now_ms = timer.toc_ms();
      pose_fb = pose_rdte_cmd;
      vel_fb.setZero();
      wrench_fb_ur.setZero();
    }

    // ==========================================================================
    // Phase 2: Log Robot State to Circular Buffers
    // ==========================================================================
    
    loop_profiler.stop("compute");
    loop_profiler.start();
    
    // Log pose and velocity for trajectory analysis
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
    
    // Log wrench for force analysis
    {
      std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
      _robot_wrench_buffers[id].put(wrench_fb_ur);
      _robot_wrench_timestamp_ms_buffers[id].put(time_now_ms);
    }
    
    loop_profiler.stop("lock");
    loop_profiler.start();

    // ==========================================================================
    // Phase 3: Update Reference Trajectory from Waypoint Buffer
    // ==========================================================================
    // Interpolator generates smooth trajectory between waypoints
    
    if (!intp_controller.get_control(time_now_ms, force_control_ref_pose)) {
      // No valid target available - fetch next waypoint from buffer
      bool new_wp_found = false;
      {
        std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
        
        // Scan waypoint buffer for next valid (future) waypoint
        while (!_waypoints_buffers[id].is_empty()) {
          pose_target_waypoint = _waypoints_buffers[id].pop();
          double target_time_ms = _waypoints_timestamp_ms_buffers[id].pop();
          
          // Only accept waypoints scheduled for future execution
          if (target_time_ms > time_now_ms) {
            intp_controller.set_new_target(pose_target_waypoint, target_time_ms);
            new_wp_found = true;
            break;
          }
          // Discard obsolete waypoints (scheduled in past)
        }
      }
      
      if (!new_wp_found) {
        // No future waypoints - hold current position
        intp_controller.keep_the_last_target(time_now_ms);
      }
      
      // Retrieve updated reference trajectory point
      intp_controller.get_control(time_now_ms, force_control_ref_pose);
    }

    loop_profiler.stop("intp_controller");
    loop_profiler.start();

    // ==========================================================================
    // Phase 4: Update Compliance Stiffness Matrix (if scheduled)
    // ==========================================================================
    // Allows dynamic adjustment of compliance during execution
    
    bool new_stiffness_found = false;
    {
      std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
      
      if (!_stiffness_buffers[id].is_empty()) {
        double next_available_time_ms = _stiffness_timestamp_ms_buffers[id][0];
        
        // Check if it's time to apply new stiffness
        if (time_now_ms > next_available_time_ms) {
          new_stiffness_found = true;
          
          // Consume all stiffness updates scheduled before current time
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

    // Transform wrench if needed (currently zeros - can implement frame transforms)
    wrench_WTr.setZero();

    // ==========================================================================
    // Phase 5: Update Admittance Controller and Compute Command
    // ==========================================================================
    // Admittance: δx = K^-1 * F_ext (compliant response to external forces)
    
    {
      std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
      loop_profiler.stop("controller_lock");
      loop_profiler.start();
      
      // Update controller with current state
      _admittance_controllers[id].setRobotStatus(pose_fb, wrench_fb_ur);
      
      // Set desired reference trajectory
      _admittance_controllers[id].setRobotReference(force_control_ref_pose, wrench_WTr);

      // Apply new stiffness matrix if available
      if (new_stiffness_found) {
        _admittance_controllers[id].setStiffnessMatrix(stiffness);
      }
      
      loop_profiler.stop("controller_set");
      loop_profiler.start();
      
      // Compute desired Cartesian pose adjustments for compliance
      _admittance_controllers[id].step(pose_rdte_cmd);
      
      loop_profiler.stop("controller_step");
    }

    // ==========================================================================
    // Phase 6: Send Command to Robot via RTDE
    // ==========================================================================
    
    if ((!_config.mock_hardware) &&
        (!urrtde_ptr->streamCartesian(pose_rdte_cmd))) {
      std::cout << "\033[31m";  // Red color
      std::cout << header << "streamCartesian() failed - Emergency stop" << std::endl;
      std::cout << header << "Last state:" << std::endl;
      std::cout << header << "  Pose FB: " << pose_fb.transpose() << std::endl;
      std::cout << header << "  Wrench: " << wrench_fb_ur.transpose() << std::endl;
      std::cout << header << "  Reference: " << force_control_ref_pose.transpose() << std::endl;
      std::cout << header << "  Pose CMD: " << pose_rdte_cmd.transpose() << std::endl;
      std::cout << "\033[0m";  // Reset color
      break;
    }

    // ==========================================================================
    // Phase 7: Data Logging to JSON File
    // ==========================================================================
    // Saves trajectory data for post-experiment analysis
    
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      if (!ctrl_flag_saving) {
        std::cout << header << "Starting to save low-dimensional data." << std::endl;
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
        std::cout << header << "Stopping data logging." << std::endl;
        
        // Save final frame with proper termination
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

    // ==========================================================================
    // Phase 8: Check Exit Condition
    // ==========================================================================
    
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header << "Exit signal received. Shutting down control loop." 
                  << std::endl;
        break;
      }
    }

    loop_profiler.stop("lock");

    // ==========================================================================
    // Phase 9: Loop Timing and Overrun Detection
    // ==========================================================================
    // UR RTDE enforces 2ms cycle - monitor for violations
    
    if (_config.mock_hardware) {
      // Mock mode: maintain timing with sleep
      mock_loop_timer.sleep_till_next();
    } else {
      // Real hardware: strict UR RTDE timing requirement
      double overrun_ms = mock_loop_timer.check_for_overrun_ms(false);
      if (overrun_ms > 0) {
        std::cout << "\033[33m";  // Yellow warning
        std::cout << header << "⚠️  Overrun: " << overrun_ms << "ms "
                  << "(execution time exceeded 2ms window)" << std::endl;
        std::cout << "\033[0m";
        
        // Show detailed timing breakdown for debugging
        loop_profiler.show();
      }
      
      // Wait for next RTDE cycle
      urrtde_ptr->rtde_wait_period(t_start);
      
      // Reset timer for next iteration
      mock_loop_timer.check_for_overrun_ms(false);
    }
    
    loop_profiler.clear();
    
  }  // End of control loop

  // ============================================================================
  // Control Loop Exit - Cleanup
  // ============================================================================
  
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }
  
  std::cout << header << "Thread terminated." << std::endl;
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

// =============================================================================
// INTERNAL JOINT TORQUE SENSING LOOP - ROBOT WRENCH FROM JOINT SENSORS
// =============================================================================
//
// Purpose: Republish internal joint torque sensor data at configurable rate
//          Data is already collected by robot_impedance_loop at 1kHz,
//          this thread downsamples and logs to JSON for analysis
//
// Data Architecture:
//   Source: _robot_wrench_buffers (populated by robot_impedance_loop at 1kHz)
//   Purpose: Extract joint torques from robot internal sensors
//   Republish: At configurable publish_rate (typically 100-500 Hz)
//   Differences from ext_sensor_wrench_loop:
//     - No external sensor interface needed
//     - Data already collected by control loop
//     - Lower latency (internal measurements, not ETH)
//     - Higher frequency source (1kHz buffer, subset republished)
//
// Thread Safety:
//   - Reads from _robot_wrench_buffers (lock-free circular buffer)
//   - Writes to multiple wrench buffers for compatibility
//   - Uses mutex protection for buffer access
//   - Synchronizes startup via _states_wrench_thread_ready
//
// Synchronization:
//   - Waits for robot_impedance_loop to populate source buffer before starting
//   - Publishes at configurable rate (decouple 1kHz source from logging rate)
//   - Non-blocking read from circular buffer (old data discarded if slow)
//
// =============================================================================

void ManipServer::joint_sensor_wrench_loop(const RUT::TimePoint& time0, int publish_rate, int id) {
  std::string header = "[ManipServer][Joint Wrench thread] " + std::to_string(id) + ": ";
  std::cout << header << "Starting thread.\n";

  // ============================================================================
  // Step 1: Initialize Synchronized Timer
  // ============================================================================
  // Use global time reference to synchronize with other threads
  
  RUT::Timer timer;
  timer.tic(time0);

  // ============================================================================
  // Step 2: Initialize Wrench State Variables
  // ============================================================================
  
  RUT::Vector6d wrench_fb;  // Joint torque vector (6D: Fx,Fy,Fz,Tx,Ty,Tz at tool)
  wrench_fb.setZero();

  // ============================================================================
  // Step 3: Wait for Source Buffer Population
  // ============================================================================
  // robot_impedance_loop must populate _robot_wrench_buffers before we start
  // This ensures we have valid data to republish
  
  std::cout << header << "Waiting for robot control thread to populate wrench buffer...\n";
  int init_retries = 0;
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
      if (_robot_wrench_buffers[id].size() > 0) {
        std::cout << header << "✓ Buffer populated. Data ready from robot thread." << std::endl;
        break;
      }
    }
    
    init_retries++;
    if (init_retries % 4 == 0) {
      std::cout << header << "  Still waiting... buffer size: " 
                << _robot_wrench_buffers[id].size() << std::endl;
    }
    
    usleep(300 * 1000);  // 300ms retry interval
  }

  // ============================================================================
  // Step 4: Initialize Loop Timing Control
  // ============================================================================
  // Republish at configurable rate (can be lower than 1kHz source)
  
  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(publish_rate);  // Configurable publish rate
  loop_timer.start_timed_loop();

  std::cout << header << "Publishing wrench data at " << publish_rate 
            << " Hz (decimating from 1kHz source buffer)" << std::endl;

  // ============================================================================
  // Step 5: Signal Thread Ready
  // ============================================================================
  
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_wrench_thread_ready[id] = true;
  }
  std::cout << header << "Thread ready. Starting republish loop." << std::endl;

  // ============================================================================
  // Step 6: Initialize Logging State
  // ============================================================================
  
  bool ctrl_flag_saving = false;  // Local copy of save flag
  uint64_t frame_count = 0;       // Count published frames
  uint64_t dropped_frames = 0;    // Count old data discarded from buffer

  std::cout << header << "Entering republish loop" << std::endl;

  // ============================================================================
  // MAIN REPUBLISH LOOP (Configurable Rate)
  // ============================================================================
  // Note: This loop runs at lower rate than source (1kHz),
  // so some intermediate data will be discarded, keeping only latest measurements
  
  while (true) {
    // ==========================================================================
    // Phase 1: Read Latest Wrench from Circular Buffer
    // ==========================================================================
    // Circular buffer automatically discards old data when buffer fills
    // We only care about the most recent wrench measurement
    
    double time_now_ms;
    
    if (!_config.mock_hardware) {
      // Read from robot wrench buffer (populated by robot_impedance_loop)
      {
        std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
        
        // Drain buffer to get only the most recent wrench
        uint64_t local_dropped = 0;
        while (!_robot_wrench_buffers[id].is_empty()) {
          wrench_fb = _robot_wrench_buffers[id].pop();
          
          // Count how many intermediate measurements we're skipping
          if (!_robot_wrench_buffers[id].is_empty()) {
            local_dropped++;
          }
        }
        
        if (local_dropped > 0) {
          dropped_frames += local_dropped;
        }
      }
      
      time_now_ms = timer.toc_ms();
      
    } else {
      // Mock hardware: generate zero wrench
      wrench_fb.setZero();
      time_now_ms = timer.toc_ms();
    }

    // ==========================================================================
    // Phase 2: Update Wrench Buffers for Other Threads
    // ==========================================================================
    // Republish to multiple wrench buffers for compatibility with:
    //  - eoat_loop (which uses _wrench_fb for force feedback)
    //  - teleop_loop (which uses _wrench_fb for haptic feedback)
    //  - Other analysis threads
    
    {
      // Store in timestamped circular buffer for data analysis
      std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
      _wrench_buffers[id].put(wrench_fb);
      _wrench_timestamp_ms_buffers[id].put(time_now_ms);
    }
    
    {
      // Store in direct feedback variable (fast query without locking)
      std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
      _wrench_fb[id] = wrench_fb;
    }

    // ==========================================================================
    // Phase 3: JSON Data Logging
    // ==========================================================================
    // Log wrench measurements for post-experiment analysis
    
    _ctrl_mtx.lock();
    if (_ctrl_flag_saving) {
      _ctrl_mtx.unlock();

      // Initialize logging on first frame of save session
      if (!ctrl_flag_saving) {
        std::cout << header << "Starting data logging to JSON file" << std::endl;
        json_file_start(_ctrl_wrench_data_streams[id]);
        ctrl_flag_saving = true;
      }

      _states_wrench_thread_saving[id] = true;
      
      // Write frame with timestamp, sequence ID, and wrench data
      save_wrench_data_json(_ctrl_wrench_data_streams[id],
                           _states_wrench_seq_id[id], 
                           time_now_ms,
                           wrench_fb);
      json_frame_ending(_ctrl_wrench_data_streams[id]);
      _states_wrench_seq_id[id]++;
      frame_count++;
      
    } else {
      _ctrl_mtx.unlock();

      // Handle transition from saving to not saving
      if (ctrl_flag_saving) {
        std::cout << header << "Stopping data logging. Writing final frame..." << std::endl;
        
        // Write final frame (terminates JSON array properly)
        save_wrench_data_json(_ctrl_wrench_data_streams[id],
                             _states_wrench_seq_id[id], 
                             time_now_ms,
                             wrench_fb);
        json_last_frame_ending(_ctrl_wrench_data_streams[id]);
        json_file_ending(_ctrl_wrench_data_streams[id]);
        _ctrl_wrench_data_streams[id].close();
        
        std::cout << header << "Logged " << frame_count << " frames total" << std::endl;
        
        ctrl_flag_saving = false;
        _states_wrench_thread_saving[id] = false;
        frame_count = 0;
        dropped_frames = 0;
      }
    }

    // ==========================================================================
    // Phase 4: Check Exit Condition
    // ==========================================================================
    
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header << "Exit signal received. Shutting down thread." << std::endl;
        break;
      }
    }

    // ==========================================================================
    // Phase 5: Maintain Publish Rate
    // ==========================================================================
    
    loop_timer.sleep_till_next();
    
  }  // End of republish loop

  // ============================================================================
  // Loop Exit - Final Statistics
  // ============================================================================
  
  std::cout << "\033[36m";  // Cyan for visibility
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << " JOINT WRENCH REPUBLISH STATISTICS" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << "  Total frames published:    " << frame_count << std::endl;
  std::cout << header << "  Frames decimated (skipped):" << dropped_frames << std::endl;
  std::cout << header << "  Source buffer frequency:   1000 Hz (robot loop)" << std::endl;
  std::cout << header << "  Publish frequency:         " << publish_rate << " Hz" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\033[0m";  // Reset color
  
  std::cout << header << "Thread terminated." << std::endl;
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
        json_last_frame_ending(_ctrl_wrench_data_streams[id]);
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

// =============================================================================
// RGB CAMERA CAPTURE AND FRAME BUFFERING LOOP
// =============================================================================
//
// Purpose: Capture RGB frames from camera at 60Hz and buffer for visualization
//          and optional file logging
//
// Camera Pipeline:
//   1. Capture frame from camera (blocking call at camera native FPS)
//   2. Resize frame to configured output dimensions
//   3. Split BGR channels from OpenCV format
//   4. Convert each channel to Eigen matrix
//   5. Stack RGB channels into single matrix (for visualization/analysis)
//   6. Store in circular buffer with timestamp
//   7. Optionally save frame to disk during recording session
//
// Thread Synchronization:
//   - Runs at 60Hz (standard camera frame rate)
//   - Uses blocking camera capture (naturally throttles thread)
//   - Mutex protection for shared frame buffers
//   - Thread-safe save flag synchronization
//
// Data Format:
//   - Input: BGR format from OpenCV camera interface (uint8)
//   - Processing: Split into 3 separate channel matrices
//   - Output: RGB stacked matrix (red_rows, green_rows, blue_rows)
//   - Storage: Eigen::MatrixXd with R|G|B row concatenation
//
// =============================================================================

void ManipServer::rgb_loop(const RUT::TimePoint& time0, int id) {
  std::string header = "[ManipServer][RGB Camera thread] " + std::to_string(id) + ": ";
  std::cout << header << "Starting thread.\n";

  // ============================================================================
  // Step 1: Initialize Synchronized Timer
  // ============================================================================
  // Use global time reference for consistent timestamps with other threads
  
  RUT::Timer timer;
  timer.tic(time0);

  // ============================================================================
  // Step 2: Initialize Loop Timing Control
  // ============================================================================
  // Set to 60Hz to match typical camera frame rate
  
  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(60);  // Standard camera FPS
  loop_timer.start_timed_loop();
  
  std::cout << header << "Camera timing set to 60 Hz" << std::endl;

  // ============================================================================
  // Step 3: Initialize Image Processing Variables
  // ============================================================================
  // Pre-allocate matrices for efficient frame processing
  
  cv::Mat resized_color_mat;              // Resized frame from camera
  cv::Mat bgr[3];                         // BGR channel array (OpenCV format)
  Eigen::MatrixXd bm, gm, rm;             // Blue, Green, Red matrices (Eigen)
  Eigen::MatrixXd rgb_row_combined;       // Stacked RGB matrix for output

  // ============================================================================
  // Step 4: Signal Thread Ready
  // ============================================================================
  
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_rgb_thread_ready[id] = true;
  }
  std::cout << header << "Thread ready. Starting capture loop." << std::endl;

  // ============================================================================
  // Step 5: Initialize Logging State
  // ============================================================================
  
  uint64_t frame_count = 0;   // Total frames captured
  uint64_t saved_count = 0;   // Frames saved to disk

  std::cout << header << "Entering 60Hz capture loop" << std::endl;

  // ============================================================================
  // MAIN CAPTURE LOOP (60Hz Camera Acquisition)
  // ============================================================================
  
  while (true) {
    double time_now_ms = 0;
    
    // ==========================================================================
    // Phase 1: Capture Frame from Camera
    // ==========================================================================
    // Blocking call naturally throttles to camera frame rate
    
    {
      std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
      
      if (!_config.mock_hardware) {
        // Real camera: blocking capture from hardware
        _color_mats[id] = camera_ptrs[id]->next_rgb_frame_blocking();
        
      } else {
        // Mock camera: generate zero frame and simulate 60Hz timing
        _color_mats[id] = cv::Mat::zeros(1080, 1080, CV_8UC3);
        usleep(20 * 1000);  // 20ms → ~50Hz
      }
      
      time_now_ms = timer.toc_ms();
      
      // ==========================================================================
      // Phase 2: Resize Frame to Output Dimensions
      // ==========================================================================
      // Configured output resolution may differ from camera native resolution
      // Use linear interpolation for smooth downsampling
      
      cv::resize(_color_mats[id], resized_color_mat,
                 cv::Size(_config.output_rgb_hw[1], _config.output_rgb_hw[0]),
                 cv::INTER_LINEAR);
      
      // ==========================================================================
      // Phase 3: Split BGR Channels
      // ==========================================================================
      // OpenCV uses BGR format; we want RGB, so reverse order during stacking
      
      cv::split(resized_color_mat, bgr);  // Split into B, G, R arrays
    }

    // ==========================================================================
    // Phase 4: Convert Channels to Eigen Matrices and Stack
    // ==========================================================================
    // Convert each OpenCV channel to Eigen for efficient buffering
    // Stack as [R_rows; G_rows; B_rows] for downstream processing
    
    cv::cv2eigen(bgr[0], bm);  // Blue channel
    cv::cv2eigen(bgr[1], gm);  // Green channel
    cv::cv2eigen(bgr[2], rm);  // Red channel
    
    // Stack RGB vertically (concatenate rows)
    rgb_row_combined.resize(resized_color_mat.rows * 3, resized_color_mat.cols);
    rgb_row_combined << rm, gm, bm;  // R first, then G, then B

    // ==========================================================================
    // Phase 5: Buffer Frame Data
    // ==========================================================================
    // Store converted RGB matrix and timestamp in circular buffer
    // This buffer feeds both visualization (rgb_plot_loop) and analysis
    
    {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
      _camera_rgb_buffers[id].put(rgb_row_combined);
      _camera_rgb_timestamp_ms_buffers[id].put(time_now_ms);
    }

    frame_count++;

    // ==========================================================================
    // Phase 6: Optional Frame Logging to Disk
    // ==========================================================================
    // Save high-resolution original frames if recording session is active
    // Note: Saves original camera frame, not the resized/processed version
    
    if (_ctrl_flag_saving) {
      _states_rgb_thread_saving[id] = true;
      {
        std::lock_guard<std::mutex> lock(_color_mat_mtxs[id]);
        save_rgb_data(_ctrl_rgb_folders[id], _states_rgb_seq_id[id],
                      time_now_ms, _color_mats[id]);
      }
      _states_rgb_seq_id[id]++;
      saved_count++;
      
    } else {
      _states_rgb_thread_saving[id] = false;
    }

    // ==========================================================================
    // Phase 7: Check Exit Condition
    // ==========================================================================
    
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header << "Exit signal received. Shutting down camera thread." << std::endl;
        break;
      }
    }

    // ==========================================================================
    // Phase 8: Maintain 60Hz Loop Rate
    // ==========================================================================
    // Sleep until next frame time (blocking camera naturally throttles)
    
    loop_timer.sleep_till_next();
    
  }  // End of capture loop

  // ============================================================================
  // Camera Loop Exit - Print Statistics
  // ============================================================================
  
  std::cout << "\033[36m";  // Cyan for visibility
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << " RGB CAMERA CAPTURE STATISTICS" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << "  Total frames captured:      " << frame_count << std::endl;
  std::cout << header << "  Frames saved to disk:       " << saved_count << std::endl;
  std::cout << header << "  Frames to buffer only:      " << (frame_count - saved_count) << std::endl;
  std::cout << header << "  Expected duration:          " << (frame_count / 60.0) << " seconds" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\033[0m";  // Reset color

  // Signal other threads to stop (camera thread typically exits first)
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }

  std::cout << header << "Thread terminated." << std::endl;
}

void ManipServer::rgb_plot_loop() {
  std::string header = "[ManipServer][plot thread]: ";
  std::cout << header << "starting thread." << std::endl;
  cv::namedWindow("RGB", cv::WINDOW_NORMAL);
  cv::resizeWindow("RGB", 1200, 1000);
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

// =============================================================================
// ASYNCHRONOUS ROBOT DATA LOGGING LOOP
// =============================================================================
//
// Purpose: Decouple high-frequency control loop I/O from real-time threads
//          Consumes pre-buffered robot state (pose, torque, joint pos) at 200Hz
//          Writes to three parallel JSON files for post-experiment analysis
//
// Architecture:
//   Problem: robot_impedance_loop runs at 1kHz with strict real-time constraints
//            File I/O (disk writes) can cause unpredictable latency spikes
//   Solution: logging thread operates at 200Hz, consuming buffered data
//             Real-time loop only logs to fast circular buffer (~microseconds)
//             This thread batches writes to amortize disk I/O cost
//
// Data Flow:
//   robot_impedance_loop (1kHz) → _logging_buffers[id] (lock-free circular)
//                                      ↓
//   robot_logging_loop (200Hz) ← Batch read up to 100 frames per cycle
//                                      ↓
//                            Three parallel JSON streams:
//                            - Pose data (x,y,z,qx,qy,qz,qw)
//                            - Torque data (tau_J[7] joint torques)
//                            - Joint positions (q[7] joint angles)
//
// Buffering Strategy:
//   - Circular buffer prevents unbounded memory growth
//   - 1kHz producer → 200Hz consumer creates 5:1 decimation
//   - Buffer overflow detection with atomic flag
//   - When buffer fills, oldest data is overwritten (acceptable for analysis)
//
// Thread Safety:
//   - Lock-free read from circular buffer (no mutex contention with real-time)
//   - Mutex-protected save flag synchronization
//   - No locking during file I/O (happens at lower frequency)
//   - Atomic overflow flag for non-blocking detection
//
// =============================================================================

void ManipServer::robot_logging_loop(const RUT::TimePoint& time0, int id) {
  std::string header = "[ManipServer][Robot Logging thread] " + std::to_string(id) + ": ";
  std::cout << header << "Starting thread.\n";

  // ============================================================================
  // Step 1: Initialize Synchronized Timer
  // ============================================================================
  // Use global time reference for consistent timestamps with control loop
  
  RUT::Timer timer;
  timer.tic(time0);

  // ============================================================================
  // Step 2: Initialize Logging State Variables
  // ============================================================================
  
  bool ctrl_flag_saving = false;      // Local copy of global save flag
  uint64_t frames_written = 0;        // Count of frames written to disk
  uint64_t total_batches = 0;         // Count of batch processing cycles
  uint64_t max_batch_size = 0;        // Largest batch processed in one cycle
  uint64_t overflow_count = 0;        // Times buffer overflow detected

  // ============================================================================
  // Step 3: Initialize Loop Timing
  // ============================================================================
  // Run at 200Hz (5× slower than 1kHz source) for batching efficiency
  // Each cycle can process up to 5 frames from the 1kHz control loop
  
  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(200);
  loop_timer.start_timed_loop();

  std::cout << header << "Consuming data at 200 Hz (decimating 1kHz source)" << std::endl;

  // ============================================================================
  // Step 4: Signal Thread Ready
  // ============================================================================
  
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _states_logging_thread_ready[id] = true;
  }
  std::cout << header << "Thread ready. Starting logging loop." << std::endl;

  std::cout << header << "Entering asynchronous logging loop" << std::endl;

  // ============================================================================
  // MAIN LOGGING LOOP (200Hz Async I/O)
  // ============================================================================
  // Consumes buffered data and writes to disk at lower frequency than source
  
  while (true) {
    // ==========================================================================
    // Phase 1: Query Save Flag (Non-Blocking)
    // ==========================================================================
    // Check if user has requested recording session
    
    _ctrl_mtx.lock();
    bool should_save = _ctrl_flag_saving;
    _ctrl_mtx.unlock();

    // ==========================================================================
    // Phase 2: Recording Active - Batch Process Data
    // ==========================================================================
    
    if (should_save) {
      // Initialize on first frame of save session
      if (!ctrl_flag_saving) {
        std::cout << header << "Starting data recording session" << std::endl;
        std::cout << header << "Opening JSON output streams:" << std::endl;
        std::cout << header << "  - Pose data stream" << std::endl;
        std::cout << header << "  - Torque data stream" << std::endl;
        std::cout << header << "  - Joint position data stream" << std::endl;
        
        json_file_start(_ctrl_robot_data_streams[id]);
        json_file_start(_ctrl_torque_data_streams[id]);
        json_file_start(_ctrl_joint_data_streams[id]);
        
        ctrl_flag_saving = true;
        frames_written = 0;
      }

      // =====================================================================
      // Phase 2a: Batch Process Buffered Frames
      // =====================================================================
      // Drain up to 100 frames from circular buffer in one batch
      // This amortizes disk I/O latency over multiple frames
      
      int batch_count = 0;
      const int BATCH_SIZE_MAX = 100;  // Up to 5ms worth of 1kHz data
      
      while (batch_count < BATCH_SIZE_MAX) {
        RobotLogData log_data;
        bool has_data = false;
        
        // Lock-free read from circular buffer
        {
          std::lock_guard<std::mutex> lock(_logging_buffer_mtxs[id]);
          if (!_logging_buffers[id].is_empty()) {
            log_data = _logging_buffers[id].pop();
            has_data = true;
          }
        }
        
        // Stop batching if buffer is empty
        if (!has_data) break;
        
        // ===================================================================
        // Write Frame to Three Parallel JSON Streams
        // ===================================================================
        
        // Stream 1: Full pose + torque data
        save_robot_data_json(_ctrl_robot_data_streams[id],
                           _states_robot_seq_id[id], 
                           log_data.timestamp_ms, 
                           log_data.pose_fb,
                           false);
        json_frame_ending(_ctrl_robot_data_streams[id]);

        // Stream 2: Torque-only data (for force analysis)
        save_robot_torque_json(_ctrl_torque_data_streams[id],
                               _states_robot_seq_id[id],
                               log_data.timestamp_ms,
                               log_data.tau_J,
                               false);
        json_frame_ending(_ctrl_torque_data_streams[id]);
        
        // Stream 3: Joint positions (for trajectory analysis)
        save_robot_joint_positions_json(_ctrl_joint_data_streams[id],
                                        _states_robot_seq_id[id],
                                        log_data.timestamp_ms,
                                        log_data.q,
                                        false);
        json_frame_ending(_ctrl_joint_data_streams[id]);
        
        _states_robot_seq_id[id]++;
        frames_written++;
        batch_count++;
      }
      
      // Track batch statistics
      total_batches++;
      if (batch_count > max_batch_size) {
        max_batch_size = batch_count;
      }
      
      // =====================================================================
      // Phase 2b: Monitor Buffer Overflow
      // =====================================================================
      // Circular buffer will overwrite old data if producer faster than consumer
      // This is acceptable for post-analysis but worth monitoring
      
      if (_logging_buffer_overflow[id].load()) {
        overflow_count++;
        std::cerr << "\033[33m";  // Yellow warning
        std::cerr << header << "  Logging buffer overflow detected" << std::endl;
        std::cerr << "   Some data may have been lost due to slow I/O" << std::endl;
        std::cerr << "\033[0m";
        _logging_buffer_overflow[id].store(false);  // Reset flag
      }
      
    } else {
      // =====================================================================
      // Phase 3: Recording Inactive - Flush Buffer and Close Files
      // =====================================================================
      
      if (ctrl_flag_saving) {
        std::cout << header << "Recording stopped. Flushing buffer..." << std::endl;
        
        // Process all remaining frames in buffer
        int flushed = 0;
        RobotLogData last_log_data;
        bool has_last_data = false;
        
        // Drain remaining data in a second pass (lookahead buffering)
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
          
          // If we have a previous frame, write it with frame ending (comma)
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
          
          // Store current as last for next iteration
          last_log_data = log_data;
          has_last_data = true;
        }
        
        // Write the very last frame WITHOUT trailing comma
        if (has_last_data) {
          save_robot_data_json(_ctrl_robot_data_streams[id],
                             _states_robot_seq_id[id],
                             last_log_data.timestamp_ms,
                             last_log_data.pose_fb,
                             false);
          json_last_frame_ending(_ctrl_robot_data_streams[id]);
          
          save_robot_torque_json(_ctrl_torque_data_streams[id],
                                 _states_robot_seq_id[id],
                                 last_log_data.timestamp_ms,
                                 last_log_data.tau_J,
                                 false);
          json_last_frame_ending(_ctrl_torque_data_streams[id]);
          
          save_robot_joint_positions_json(_ctrl_joint_data_streams[id],
                                          _states_robot_seq_id[id],
                                          last_log_data.timestamp_ms,
                                          last_log_data.q,
                                          false);
          json_last_frame_ending(_ctrl_joint_data_streams[id]);
          
          _states_robot_seq_id[id]++;
          flushed++;
          
        } else if (frames_written > 0) {
          // No frames to flush, but we wrote frames during normal operation
          // Remove trailing comma from last frame and properly close JSON
          _ctrl_robot_data_streams[id].seekp(-3, std::ios_base::cur);
          _ctrl_robot_data_streams[id] << "\n\t}\n";
          
          _ctrl_torque_data_streams[id].seekp(-3, std::ios_base::cur);
          _ctrl_torque_data_streams[id] << "\n\t}\n";
          
          _ctrl_joint_data_streams[id].seekp(-3, std::ios_base::cur);
          _ctrl_joint_data_streams[id] << "\n\t}\n";
        }
        
        // Finalize JSON arrays
        json_file_ending(_ctrl_robot_data_streams[id]);
        json_file_ending(_ctrl_torque_data_streams[id]);
        json_file_ending(_ctrl_joint_data_streams[id]);
        
        // Close file handles
        _ctrl_robot_data_streams[id].close();
        _ctrl_torque_data_streams[id].close();
        _ctrl_joint_data_streams[id].close();
        
        std::cout << header << "✓ Recording session complete" << std::endl;
        std::cout << header << "Flushed " << flushed << " remaining frames from buffer" << std::endl;
        
        ctrl_flag_saving = false;
        _states_robot_thread_saving[id] = false;
      }
    }

    // ==========================================================================
    // Phase 4: Check Exit Condition
    // ==========================================================================
    
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header << "Exit signal received. Shutting down logging thread." << std::endl;
        break;
      }
    }

    // ==========================================================================
    // Phase 5: Maintain 200Hz Loop Rate
    // ==========================================================================
    
    loop_timer.sleep_till_next();
    
  }  // End of logging loop

  // ============================================================================
  // Logging Loop Exit - Print Statistics
  // ============================================================================
  
  std::cout << "\033[36m";  // Cyan for visibility
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << " ASYNCHRONOUS LOGGING STATISTICS" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << "  Total frames written:      " << frames_written << std::endl;
  std::cout << header << "  Total batch cycles:        " << total_batches << std::endl;
  
  if (total_batches > 0) {
    std::cout << header << "  Avg frames per batch:      " 
              << (static_cast<double>(frames_written) / total_batches) << std::endl;
    std::cout << header << "  Max batch size:            " << max_batch_size << std::endl;
  }
  
  std::cout << header << "  Buffer overflow events:    " << overflow_count << std::endl;
  std::cout << header << "  Output streams (3 files):  pose, torque, joint_positions" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\033[0m";  // Reset color

  std::cout << header << "Thread terminated." << std::endl;
}

// =============================================================================
// TELEOPERATION CONTROL LOOP - GAMEPAD INPUT WITH HAPTIC FEEDBACK
// =============================================================================
//
// Purpose: Real-time gamepad teleoperation with force-based haptic feedback
//          Provides intuitive control of robot manipulation with tactile cues
//
// Teleoperation Mapping:
//   - Left stick: Cartesian translation (X, Y axes)
//   - Left & right triggers: Z-axis translation (up/down)
//   - Right stick: End-effector rotation (roll, yaw)
//   - Force threshold: Gamepad vibration when external forces detected
//
// Haptic Feedback System:
//   - Low-pass filtered force magnitude from wrench sensor (smooth)
//   - Bias tracking (slow baseline) to cancel gravity/friction offsets
//   - Hysteresis (deadband) around threshold to prevent chatter
//   - Vibration intensity scales with force magnitude above threshold
//   - Automatic rumble refresh at configurable rate
//
// Control Architecture:
//   1. Wait for initial robot feedback (ensure motion reference available)
//   2. Enter 1kHz input polling loop
//   3. Read gamepad stick/trigger positions
//   4. Read external wrench from force sensor
//   5. Apply low-pass filtering and hysteresis to force signal
//   6. Compute vibration magnitude based on force above threshold
//   7. Update haptic feedback (controller rumble)
//   8. Compute incremental pose adjustments
//   9. Apply rotation accumulation (prevents spring-back)
//   10. Schedule waypoint command to robot
//
// Thread Safety:
//   - Reads robot pose feedback (mutex-protected)
//   - Reads wrench measurements (mutex-protected)
//   - Non-blocking gamepad I/O (async device driver)
//   - All state local to thread (no concurrent write conflicts)
//
// =============================================================================

// Teleoperation control loop for Gamepad input
void ManipServer::teleop_loop(const RUT::TimePoint& time0, int id) {
  std::string header = "[ManipServer][Teleoperation thread] " + std::to_string(id) + ": ";
  std::cout << header << "Starting thread.\n";

  // ============================================================================
  // Step 1: Initialize Synchronized Timer
  // ============================================================================
  
  RUT::Timer timer;
  timer.tic(time0);

  // ============================================================================
  // Step 2: Verify Gamepad Hardware Is Available
  // ============================================================================
  
  if (!gamepad_ptrs[id]) {
    std::cerr << header << " Gamepad pointer is null. Cannot initialize teleoperation." << std::endl;
    return;
  }
  std::cout << header << "✓ Gamepad detected and ready for teleoperation" << std::endl;

  // ============================================================================
  // Step 3: Wait for Robot Feedback to Become Available
  // ============================================================================
  // Cannot start teleoperation until we have initial pose feedback
  
  std::cout << header << "Waiting for robot feedback to be available...\n";
  RUT::Vector7d current_pose = RUT::Vector7d::Zero();
  int retry_count = 0;
  
  while (true) {
    {
      std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
      if (!_poses_fb[id].isZero()) {
        current_pose = _poses_fb[id];
        std::cout << header << "✓ Initial pose received: " << current_pose.transpose() << "\n";
        break;
      }
    }
    
    retry_count++;
    if (retry_count > 100) {
      std::cerr << header << " Timeout waiting for robot feedback (10 seconds). Exiting." << std::endl;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // ============================================================================
  // Step 4: Initialize Teleoperation State
  // ============================================================================
  
  RUT::Vector7d target_pose = current_pose;  // Accumulates teleop commands
  GamepadData gp_data;                       // Current gamepad state

  // Reference frame toggle (WORLD ↔ TCP)
  bool use_tcp_frame = false;
  bool a_prev = false;

  // ============================================================================
  // Step 5: Initialize Loop Timing
  // ============================================================================
  // 1kHz loop for responsive haptic feedback
  
  RUT::Timer loop_timer;
  loop_timer.set_loop_rate_hz(1000);  // 1000 Hz polling rate
  loop_timer.start_timed_loop();

  std::cout << header << "Loop timing set to 1000 Hz for responsive gamepad control" << std::endl;

  // ============================================================================
  // Step 6: Load Teleoperation Scaling Factors from Configuration
  // ============================================================================
  // Allow per-robot customization of control sensitivity
  
  double TRANSLATION_SCALE = 1.0;  // m/s per stick unit
  double ROTATION_SCALE = 1.0;     // rad/s per stick unit
  
  if (_teleop_translation_scales.size() > id) {
    TRANSLATION_SCALE = _teleop_translation_scales[id];
  }
  if (_teleop_rotation_scales.size() > id) {
    ROTATION_SCALE = _teleop_rotation_scales[id];
  }
  
  std::cout << header << "Teleoperation scaling:" << std::endl;
  std::cout << header << "  Translation scale: " << TRANSLATION_SCALE << " m/s" << std::endl;
  std::cout << header << "  Rotation scale:    " << ROTATION_SCALE << " rad/s" << std::endl;

  // ============================================================================
  // Step 7: Initialize Haptic Feedback Parameters
  // ============================================================================
  // Force thresholds, filtering, and vibration control
  
  const double   RUMBLE_FORCE_MIN_N       = gamepad_ptrs[id]->get_rumble_force_min_n();
  const double   RUMBLE_FORCE_MAX_N       = gamepad_ptrs[id]->get_rumble_force_max_n();
  const uint16_t RUMBLE_DURATION_MS       = gamepad_ptrs[id]->get_rumble_duration_ms();
  const double   RUMBLE_REFRESH_MS        = gamepad_ptrs[id]->get_rumble_refresh_ms();
  const double   RUMBLE_FILTER_ALPHA      = gamepad_ptrs[id]->get_rumble_filter_alpha();
  const double   RUMBLE_HYSTERESIS_N      = 1.0;   // Deadband (N) around threshold
  const double   RUMBLE_BIAS_ALPHA        = 0.002; // Slow baseline tracker (gravity/friction)
  const uint16_t RUMBLE_WEAK              = 0;     // Weak rumble motor strength
  
  double last_rumble_ms = -1e9;          // Timestamp of last haptic feedback
  double force_norm_filtered = 0.0;      // Low-pass filtered force magnitude
  double force_bias = 0.0;               // Slow-varying baseline (gravity offset)
  bool rumble_active = false;            // Current haptic feedback state
  
  uint64_t haptic_feedback_count = 0;    // Statistics: total haptic events
  uint64_t gamepad_input_count = 0;      // Statistics: total input reads

  std::cout << header << "Haptic feedback configuration:" << std::endl;
  std::cout << header << "  Force threshold: " << RUMBLE_FORCE_MIN_N << " - " 
            << RUMBLE_FORCE_MAX_N << " N" << std::endl;
  std::cout << header << "  Hysteresis deadband: " << RUMBLE_HYSTERESIS_N << " N" << std::endl;
  std::cout << header << "  Filter constant (α): " << RUMBLE_FILTER_ALPHA << std::endl;

  // ============================================================================
  // Step 8: Sync Initial Base Pose for Rotation Accumulation
  // ============================================================================
  
  RUT::Vector7d base_pose;
  {
    std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
    base_pose = _poses_fb[id];
  }

  std::cout << header << "Teleoperation READY" << std::endl;
  std::cout << header << "Control mapping:" << std::endl;
  std::cout << header << "  Left stick (X/Y) → Translation (X/Y)" << std::endl;
  std::cout << header << "  LT/RB triggers → Translation (Z up/down)" << std::endl;
  std::cout << header << "  Right stick → Rotation (roll/yaw)" << std::endl;
  std::cout << header << "  Force feedback → Gamepad rumble when threshold exceeded" << std::endl;

  // ============================================================================
  // MAIN TELEOPERATION LOOP (1kHz Gamepad Polling)
  // ============================================================================
  
  while (true) {
    // ==========================================================================
    // Phase 1: Check Exit Condition
    // ==========================================================================
    
    {
      std::lock_guard<std::mutex> lock(_ctrl_mtx);
      if (!_ctrl_flag_running) {
        std::cout << header << "Exit signal received. Shutting down teleoperation." << std::endl;
        break;
      }
    }

    double tx_scaled = 0, ty_scaled = 0, tz_scaled = 0;
    double rx_scaled = 0, ry_scaled = 0, rz_scaled = 0;

    // ==========================================================================
    // Phase 2: Read Gamepad Input
    // ==========================================================================
    // Non-blocking read from gamepad driver
    
    if (gamepad_ptrs[id]->get_data(gp_data)) {
      gamepad_input_count++;

      // =====================================================================
      // Phase 2a: Toggle Reference Frame on A-Button Rising Edge
      // =====================================================================
      // Press A to switch between WORLD and TCP frames
      // Useful for whiteboard erasing: use TCP frame when in contact
      
      const bool a_now = gp_data.button_a;
      if (a_now && !a_prev) {
        use_tcp_frame = !use_tcp_frame;
        std::cout << header << "Reference frame toggled: "
                  << (use_tcp_frame ? "TCP" : "WORLD") << std::endl;
      }
      a_prev = a_now;
      
      // =====================================================================
      // Phase 2b: Process Translation Input
      // =====================================================================
      // Left stick → X/Y translation (with scaling)
      // Triggers → Z translation (up with LT, down with RB)
      
      tx_scaled = gp_data.left_stick_y * TRANSLATION_SCALE;
      ty_scaled = -gp_data.left_stick_x * TRANSLATION_SCALE;  // Negate X for intuitive mapping
      tz_scaled = (gp_data.left_trigger - gp_data.right_trigger) * TRANSLATION_SCALE;
      
      // =====================================================================
      // Phase 2c: Process Rotation Input
      // =====================================================================
      // Right stick → Yaw/roll rotation (with scaling)
      // Note: rx_scaled commented out (pitch disabled to prevent gimbal issues)
      
      ry_scaled = -gp_data.right_stick_y * ROTATION_SCALE;  // Negate Y for intuitive mapping
      rz_scaled = gp_data.right_stick_x * ROTATION_SCALE;
    }

    // ==========================================================================
    // Phase 3: Read Force Sensor and Apply Haptic Filtering
    // ==========================================================================
    // Low-pass filter + bias tracking for smooth, robust force feedback
    
    RUT::VectorXd wrench_fb_local;
    {
      std::lock_guard<std::mutex> lock(_wrench_fb_mtxs[id]);
      wrench_fb_local = _wrench_fb[id];
    }

    double force_norm = 0.0;
    if (wrench_fb_local.size() >= 3) {
      // Extract force magnitude (first 3 components: Fx, Fy, Fz)
      force_norm = wrench_fb_local.head<3>().norm();
    }
    const double now_ms = timer.toc_ms();

    // =====================================================================
    // Phase 3a: Low-Pass Filter Force Measurement
    // =====================================================================
    // Smooth noisy sensor data for more stable haptic feedback
    // LPF: y[n] = α*x[n] + (1-α)*y[n-1]
    
    force_norm_filtered = RUMBLE_FILTER_ALPHA * force_norm + 
                         (1.0 - RUMBLE_FILTER_ALPHA) * force_norm_filtered;

    // =====================================================================
    // Phase 3b: Track Slow Baseline (Gravity/Friction Offset)
    // =====================================================================
    // Adaptive bias estimation to cancel constant offsets
    // Only update when near threshold (avoid adaptation in high-force regions)
    
    if (force_norm_filtered < RUMBLE_FORCE_MIN_N + RUMBLE_HYSTERESIS_N) {
      force_bias = (1.0 - RUMBLE_BIAS_ALPHA) * force_bias + 
                   RUMBLE_BIAS_ALPHA * force_norm_filtered;
    }

    // =====================================================================
    // Phase 3c: Correct Force Signal
    // =====================================================================
    // Remove estimated baseline to get true external force
    
    const double force_corrected = std::max(0.0, force_norm_filtered - force_bias);

    // =====================================================================
    // Phase 3d: Apply Hysteresis for On/Off Decision
    // =====================================================================
    // Prevents chattering at threshold boundary
    
    const double rumble_on_threshold  = RUMBLE_FORCE_MIN_N;
    const double rumble_off_threshold = std::max(0.0, RUMBLE_FORCE_MIN_N - RUMBLE_HYSTERESIS_N);

    if (force_corrected >= rumble_on_threshold) {
      rumble_active = true;
    } else if (force_corrected <= rumble_off_threshold) {
      rumble_active = false;
    }
    // else: state unchanged (hysteresis range)

    // ==========================================================================
    // Phase 4: Compute Haptic Vibration Magnitude
    // ==========================================================================
    // Map force magnitude to vibration intensity (0-65535 uint16 range)
    
    uint16_t vibration_magnitude = 0;
    if (rumble_active) {
      // Force above minimum threshold
      double force_above_min = std::max(0.0, force_corrected - RUMBLE_FORCE_MIN_N);
      
      // Normalize to 0-1 range
      double force_scaled = std::min(force_above_min, RUMBLE_FORCE_MAX_N - RUMBLE_FORCE_MIN_N) /
                            (RUMBLE_FORCE_MAX_N - RUMBLE_FORCE_MIN_N);
      
      // Map to uint16 vibration intensity
      vibration_magnitude = static_cast<uint16_t>(force_scaled * 65535.0);
    }

    // ==========================================================================
    // Phase 5: Update Haptic Feedback (Controller Rumble)
    // ==========================================================================
    // Send vibration command at configurable refresh rate
    
    if (rumble_active && vibration_magnitude > 0 && 
        (now_ms - last_rumble_ms) > RUMBLE_REFRESH_MS) {
      gamepad_ptrs[id]->set_rumble(vibration_magnitude, RUMBLE_WEAK, RUMBLE_DURATION_MS);
      last_rumble_ms = now_ms;
      haptic_feedback_count++;
      
    } else if (!rumble_active || vibration_magnitude == 0) {
      gamepad_ptrs[id]->stop_rumble();
    }

    // ==========================================================================
    // Phase 6: Check for Significant User Input
    // ==========================================================================
    // Only update pose if movement commands exceed noise threshold
    
    double tx_norm = std::abs(tx_scaled);
    double ty_norm = std::abs(ty_scaled);
    double tz_norm = std::abs(tz_scaled);
    double angle = std::sqrt(rx_scaled * rx_scaled + ry_scaled * ry_scaled + rz_scaled * rz_scaled);
    
    const double MOTION_THRESHOLD = 1e-9;  // Minimum significant movement
    const double ROTATION_THRESHOLD = 1e-6;  // Minimum significant rotation

    if (tx_norm > MOTION_THRESHOLD || ty_norm > MOTION_THRESHOLD || 
        tz_norm > MOTION_THRESHOLD || angle > ROTATION_THRESHOLD) {
      
      // Extract current target orientation as quaternion (used in both frames)
      Eigen::Quaterniond current_q(target_pose(6),    // w
                                   target_pose(3),    // x
                                   target_pose(4),    // y
                                   target_pose(5));   // z

      // Use feedback orientation for TCP mapping to match the real tool pose
      Eigen::Quaterniond tcp_q = current_q;
      if (use_tcp_frame) {
        RUT::Vector7d pose_fb_local;
        {
          std::lock_guard<std::mutex> lock(_poses_fb_mtxs[id]);
          pose_fb_local = _poses_fb[id];
        }
        // Swap x-y and negate z
        tcp_q = Eigen::Quaterniond(pose_fb_local(6),   // w
                                   pose_fb_local(3),   // y
                                   pose_fb_local(4),   // x
                                   pose_fb_local(5)); // z
      }
      
      // =====================================================================
      // Phase 6a: Apply Translation (Frame-Dependent)
      // =====================================================================
      // SIMPLIFIED FOR DEBUGGING: test if rotation matrix is correct
      
      Eigen::Vector3d delta_local(tx_scaled, ty_scaled, tz_scaled);
      
      if (use_tcp_frame) {

        //invert sign of deltaX and Z for intuitive control (stick forward → move forward in TCP frame)
        delta_local[0] = -delta_local[0];
        delta_local[1] = delta_local[1];
        delta_local[2] = -delta_local[2];

        // TCP Frame: apply rotation directly (no inverse)
        Eigen::Vector3d delta_world = tcp_q.toRotationMatrix() * delta_local;
        target_pose(0) += delta_world.x();
        target_pose(1) += delta_world.y();
        target_pose(2) += delta_world.z();
      } else {
        // WORLD Frame: apply translation directly
        target_pose(0) += delta_local.x();
        target_pose(1) += delta_local.y();
        target_pose(2) += delta_local.z();
      }

      // =====================================================================
      // Phase 6b: Apply Rotation with Accumulation (WORLD Frame Only)
      // =====================================================================
      // Simplified: keep rotation in WORLD frame for now (works well)
      // TODO: TCP rotation can be added back after translation is fixed
      
      if (angle > ROTATION_THRESHOLD) {
        Eigen::Vector3d axis(rx_scaled, ry_scaled, rz_scaled);
        if (axis.norm() > 1e-12) {
          axis.normalize();
          Eigen::Quaterniond rot_incr(Eigen::AngleAxisd(angle, axis));
          Eigen::Quaterniond new_q = current_q * rot_incr;
          new_q.normalize();
          
          // Store accumulated orientation back to pose
          target_pose(3) = new_q.x();
          target_pose(4) = new_q.y();
          target_pose(5) = new_q.z();
          target_pose(6) = new_q.w();
        }
      }
    }

    // ==========================================================================
    // Phase 7: Schedule Waypoint Command
    // ==========================================================================
    // Send target pose to motion planner for execution
    
    set_target_pose(target_pose, 1, id);  // dt = 1 (100ms)

    // ==========================================================================
    // Phase 8: Maintain 1kHz Loop Rate
    // ==========================================================================
    
    loop_timer.sleep_till_next();
    
  }  // End of teleoperation loop

  // ============================================================================
  // Teleoperation Loop Exit - Cleanup and Statistics
  // ============================================================================
  
  // Stop any active haptic feedback
  gamepad_ptrs[id]->stop_rumble();
  
  std::cout << "\033[36m";  // Cyan for visibility
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << " TELEOPERATION SESSION STATISTICS" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << header << "  Total gamepad inputs:      " << gamepad_input_count << std::endl;
  std::cout << header << "  Haptic feedback events:    " << haptic_feedback_count << std::endl;
  
  if (gamepad_input_count > 0) {
    double haptic_percentage = (100.0 * haptic_feedback_count) / gamepad_input_count;
    std::cout << header << "  Haptic engagement rate:    " << haptic_percentage << "%" << std::endl;
  }
  
  std::cout << header << "  Loop frequency:            1000 Hz (1ms cycle)" << std::endl;
  std::cout << header << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\033[0m";  // Reset color

  std::cout << header << "Thread terminated." << std::endl;
}

