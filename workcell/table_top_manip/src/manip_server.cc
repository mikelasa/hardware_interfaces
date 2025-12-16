#include "table_top_manip/manip_server.h"
#include "helpers.hpp"

ManipServer::ManipServer(const std::string& config_path) {
  initialize(config_path);
}

ManipServer::~ManipServer() {}

bool ManipServer::initialize(const std::string& config_path) {
  std::cout << "[ManipServer] Initializing.\n";

  // start timer to measure setup time
  RUT::TimePoint time0 = _timer.tic();

  // read config files
  std::cout << "[ManipServer] Reading config files.\n";
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
    _config.deserialize(config);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  // sets ID for each robot if bimanual
  if (_config.bimanual) {
    _id_list = {0, 1};
  } else {
    _id_list = {0};
  }

  std::cout << "_id_list: " << _id_list.size() << std::endl;

  // parameters to be obtained from config
  std::vector<int> wrench_publish_rate;

  std::cout << "[ManipServer] bimanual: " << _config.bimanual << std::endl;
  std::cout << "[ManipServer] Initialize each hardware interface.\n";

  // initialize hardwares
  // if not using mock hardware
  if (!_config.mock_hardware) {
    for (int id : _id_list) {
      // Robot
      if (_config.robot_selection == RobotSelection::UR_RTDE) {
        // UR Robot initialization
        URRTDE::URRTDEConfig robot_config;
        try {
          robot_config.deserialize(config["ur_rtde" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the UR robot config file: " << e.what()
                    << std::endl;
          return false;
        }
        robot_ptrs.emplace_back(new URRTDE);
        URRTDE* urrtde_ptr = static_cast<URRTDE*>(robot_ptrs[id].get());
        if (!urrtde_ptr->init(time0, robot_config)) {
          std::cerr << "Failed to initialize UR RTDE for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
      } else if (_config.robot_selection == RobotSelection::FRANKA) {
        // Franka Robot initialization
        FRANKA::FRANKAConfig robot_config;
        try {
          robot_config.deserialize(config["franka" + std::to_string(id)]);
          std::cout << "ip: " << robot_config.robot_ip << std::endl;
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the Franka robot config file: " << e.what()
                    << std::endl;
          return false;
        }
        robot_ptrs.emplace_back(new FRANKA);
        FRANKA* franka_ptr = static_cast<FRANKA*>(robot_ptrs[id].get());
        if (!franka_ptr->init(time0, robot_config)) {
          std::cerr << "Failed to initialize Franka for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
      } else {
        std::cerr << "Unsupported robot type. Exiting." << std::endl;
        return false;
      }

      // EoAT - WSG Gripper
      if (_config.run_eoat_thread) {
        // only initialize if the thread is running
        WSGGripper::WSGGripperConfig eoat_config;
        try {
          eoat_config.deserialize(config["wsg_gripper" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the eoat config file: " << e.what()
                    << std::endl;
          return false;
        }
        eoat_ptrs.emplace_back(new WSGGripper);
        WSGGripper* wsggripper_ptr =
            static_cast<WSGGripper*>(eoat_ptrs[id].get());
        if (!wsggripper_ptr->init(time0, eoat_config)) {
          std::cerr << "Failed to initialize WSGGripper for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
      }

      // Camera
      // first confirm which camera is being used
      if (_config.camera_selection == CameraSelection::GOPRO) {
        GoPro::GoProConfig gopro_config;
        try {
          gopro_config.deserialize(config["gopro" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the GoPro config file: " << e.what()
                    << std::endl;
          return false;
        }
        camera_ptrs.emplace_back(new GoPro);
        GoPro* gopro_ptr = static_cast<GoPro*>(camera_ptrs[id].get());
        if (!gopro_ptr->init(time0, gopro_config)) {
          std::cerr << "Failed to initialize GoPro for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
      // REALSENSE CAMERA
      } else if (_config.camera_selection == CameraSelection::REALSENSE) {
        Realsense::RealsenseConfig realsense_config;
        try {
          realsense_config.deserialize(
              config["realsense" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the Realsense config file: " << e.what()
                    << std::endl;
          return false;
        }
        camera_ptrs.emplace_back(new Realsense);
        Realsense* realsense_ptr =
            static_cast<Realsense*>(camera_ptrs[id].get());
        if (!realsense_ptr->init(time0, realsense_config)) {
          std::cerr << "Failed to initialize realsense for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
      } else {
        std::cerr << "Invalid camera selection. Exiting." << std::endl;
        return false;
      }

      // force sensor
      if (_config.force_sensing_mode == ForceSensingMode::FORCE_MODE_ATI) {
        ATINetft::ATINetftConfig ati_config;
        try {
          ati_config.deserialize(config["ati_netft" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the ATI Netft config file: " << e.what()
                    << std::endl;
          return false;
        }
        force_sensor_ptrs.emplace_back(new ATINetft);
        ATINetft* ati_ptr = static_cast<ATINetft*>(force_sensor_ptrs[id].get());
        if (!ati_ptr->init(time0, ati_config)) {
          std::cerr << "Failed to initialize ATI Netft for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
        wrench_publish_rate.push_back(ati_config.publish_rate);
      } else if (_config.force_sensing_mode ==
                 ForceSensingMode::FORCE_MODE_ROBOTIQ) {
        RobotiqFTModbus::RobotiqFTModbusConfig robotiq_config;
        try {
          robotiq_config.deserialize(
              config["robotiq_ft_modbus" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the Robotiq FT Modbus config file: "
                    << e.what() << std::endl;
          return false;
        }
        force_sensor_ptrs.emplace_back(new RobotiqFTModbus);
        RobotiqFTModbus* robotiq_ptr =
            static_cast<RobotiqFTModbus*>(force_sensor_ptrs[id].get());
        if (!robotiq_ptr->init(time0, robotiq_config)) {
          std::cerr << "Failed to initialize Robotiq FT Modbus for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
        wrench_publish_rate.push_back(robotiq_config.publish_rate);
      } else if (_config.force_sensing_mode ==
                 ForceSensingMode::FORCE_MODE_COINFT) {
        CoinFT::CoinFTConfig coinft_config;
        try {
          coinft_config.deserialize(config["coinft" + std::to_string(id)]);
        } catch (const std::exception& e) {
          std::cerr << "Failed to load the CoinFT config file: " << e.what()
                    << std::endl;
          return false;
        }
        force_sensor_ptrs.emplace_back(new CoinFT);
        CoinFT* coinft_ptr = static_cast<CoinFT*>(force_sensor_ptrs[id].get());
        if (!coinft_ptr->init(time0, coinft_config)) {
          std::cerr << "Failed to initialize CoinFT for id " << id
                    << ". Exiting." << std::endl;
          return false;
        }
        wrench_publish_rate.push_back(coinft_config.publish_rate);
      } else if (_config.force_sensing_mode == ForceSensingMode::JOINT_SENSORS) {
        //uses robot's internal joint torque sensors, so no config needed
        //instead create a mock force sensor
        std::cout << "[Force sensor]Using joint torque sensors for force sensing." << std::endl;

      } else {
        std::cerr << "Invalid force sensing mode. Exiting." << std::endl;
        return false;
      }
    }
  } else {
    // mock hardware, if true then the publish rate of wrench is just 7kHz
    for (int id : _id_list) {
      wrench_publish_rate.push_back(7000);
    }
  }

  // initialize Admittance controller, for each arm in id_list
  for (int id : _id_list) {
    if (_config.controller_selection == ControllerSelection::ADMITTANCE_CONTROLLER) 
    {
      // create a new instance of the AdmittanceController
    AdmittanceController::AdmittanceControllerConfig admittance_config;
    try {
      deserialize(config["admittance_controller" + std::to_string(id)],
                  admittance_config);
    } catch (const std::exception& e) {
      std::cerr << "Failed to load the admittance controller config file: "
                << e.what() << std::endl;
      return false;
    }

    // same as hardware, for each controller in id_list appends a new AdmittanceController and mutex
    _admittance_controllers.emplace_back();
    // mutex for controller, to thread safety
    _controller_mtxs.emplace_back();

    // gets the current pose of robot to use as initial pose
    //then initializes the controller with time, config parameters and pose
    RUT::Vector7d pose = RUT::Vector7d::Zero();
    if (!_config.mock_hardware) {
      robot_ptrs[id]->getCartesian(pose);
    }
    if (!_admittance_controllers[id].init(time0, admittance_config, pose)) {
      std::cerr << "Failed to initialize admittance controller for id " << id
                << ". Exiting." << std::endl;
      return false;
    }

    // set the force controlled axis, use all dofs for compliance
    RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
    // The robot should not behave with any compliance during initialization.
    // The user needs to set the desired compliance afterwards.
    int n_af = 0;
    _admittance_controllers[id].setForceControlledAxis(Tr, n_af);

    //values for stiffness and damping taken from config
    _stiffnesses_high.push_back(admittance_config.compliance6d.stiffness);
    _stiffnesses_low.push_back(RUT::Matrix6d::Zero());
    _dampings_high.push_back(admittance_config.compliance6d.damping);
    _dampings_low.push_back(_config.low_damping);
    }

    else if (_config.controller_selection == ControllerSelection::IMPEDANCE_CONTROLLER) 
    {
      // create a new instance of the ImpedanceController
      ImpedanceController::ImpedanceControllerConfig impedance_config;
      try {
        deserialize(config["impedance_controller" + std::to_string(id)],
                    impedance_config);
      } catch (const std::exception& e) {
        std::cerr << "Failed to load the impedance controller config file: "
                  << e.what() << std::endl;
        return false;
      }

      // same as hardware, for each controller in id_list appends a new ImpedanceController and mutex
    _impedance_controllers.emplace_back();
    // mutex for controller, to thread safety
    _controller_mtxs.emplace_back();

    // gets the current pose of robot to use as initial pose
    //then initializes the controller with time, config parameters and pose
    RUT::Vector7d pose = RUT::Vector7d::Zero();
    if (!_config.mock_hardware) {
      robot_ptrs[id]->getCartesian(pose);
    }
    if (!_impedance_controllers[id].init(time0, impedance_config, pose)) {
      std::cerr << "Failed to initialize impedance controller for id " << id
                << ". Exiting." << std::endl;
      return false;
    }

    // get jacobbian an state with identity matrix
    RUT::MatrixXd jacobian(6,7);
    _impedance_controllers[id].getJacobian(jacobian);
    // Force controlled axis doesnt really matter for impedance controller
    // mantain the original author implementation
    // set the force controlled axis, use all dofs for compliance
    RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
    // The robot should not behave with any compliance during initialization.
    // The user needs to set the desired compliance afterwards.
    int n_af = 0;
    _impedance_controllers[id].setForceControlledAxis(Tr, n_af);

    //values for stiffness and damping taken from config
    _stiffnesses_high.push_back(impedance_config.compliance6d.stiffness);
    _stiffnesses_low.push_back(RUT::Matrix6d::Zero());
    _dampings_high.push_back(impedance_config.compliance6d.damping);
    _dampings_low.push_back(_config.low_damping);
      
    }
  }

  // Initialize Gamepad devices for teleoperation if enabled
  std::cout << "[ManipServer] Initializing teleoperation devices.\n";
  if (_config.teleop_device == TeleopSelection::GAMEPAD) {
    for (int id : _id_list) {
      Gamepad::GamepadConfig gamepad_config;
      double translation_scale = 1e-3;   // default scale
      double rotation_scale = 1e-3;      // default scale
      
      // Parse gamepad-specific config
      auto gamepad_node = config["gamepad" + std::to_string(id)];
      if (gamepad_node) {
        if (gamepad_node["device_path"]) {
          gamepad_config.device_path = gamepad_node["device_path"].as<std::string>();
        }
        if (gamepad_node["deadzone_stick"]) {
          gamepad_config.deadzone_stick = gamepad_node["deadzone_stick"].as<double>();
        }
        if (gamepad_node["deadzone_trigger"]) {
          gamepad_config.deadzone_trigger = gamepad_node["deadzone_trigger"].as<double>();
        }
        if (gamepad_node["update_rate_hz"]) {
          gamepad_config.update_rate_hz = gamepad_node["update_rate_hz"].as<int>();
        }
        if (gamepad_node["translation_scale"]) {
          translation_scale = gamepad_node["translation_scale"].as<double>();
        }
        if (gamepad_node["rotation_scale"]) {
          rotation_scale = gamepad_node["rotation_scale"].as<double>();
        }
      }
      
      gamepad_ptrs.emplace_back(new Gamepad);
      Gamepad* gamepad_ptr = static_cast<Gamepad*>(gamepad_ptrs[id].get());
      if (!gamepad_ptr->init(gamepad_config)) {
        std::cerr << "Failed to initialize Gamepad for id " << id
                  << ". Exiting." << std::endl;
        return false;
      }
      _teleop_translation_scales.push_back(translation_scale);
      _teleop_rotation_scales.push_back(rotation_scale);
    }
  }
      

  // create the data buffers
  // each variable is saved using DataBuffer, which is a thread-safe circular buffer
  // the buffers are initialized with the appropriate sizes and names
  std::cout << "[ManipServer] Creating data buffers.\n";
  int num_ft_sensors = 1;
  for (int id : _id_list) {
    if (_config.force_sensing_mode != ForceSensingMode::JOINT_SENSORS) {
      num_ft_sensors = force_sensor_ptrs[id]->getNumSensors();
    }
    
    _camera_rgb_buffers.push_back(RUT::DataBuffer<Eigen::MatrixXd>());
    _pose_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _vel_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _eoat_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _wrench_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _robot_wrench_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _waypoints_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _eoat_waypoints_buffers.push_back(RUT::DataBuffer<Eigen::VectorXd>());
    _stiffness_buffers.push_back(RUT::DataBuffer<Eigen::MatrixXd>());

    _camera_rgb_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _pose_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _vel_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _eoat_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _wrench_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _robot_wrench_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _waypoints_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _eoat_waypoints_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());
    _stiffness_timestamp_ms_buffers.push_back(RUT::DataBuffer<double>());

    _camera_rgb_buffers[id].initialize(
        _config.rgb_buffer_size, 3 * _config.output_rgb_hw[0],
        _config.output_rgb_hw[1], "camera_rgb" + std::to_string(id));

    _pose_buffers[id].initialize(_config.robot_buffer_size, 7,
                                 1,  //xyz qwqxqyqz
                                 "pose" + std::to_string(id));
    _vel_buffers[id].initialize(_config.robot_buffer_size, 6, 1,  // xyz rxryrz
                                "vel" + std::to_string(id));
    _eoat_buffers[id].initialize(_config.eoat_buffer_size, 2, 1,  // pos, force
                                 "eoat" + std::to_string(id));
    _wrench_buffers[id].initialize(_config.wrench_buffer_size,
                                   6 * num_ft_sensors, 1,
                                   "wrench" + std::to_string(id));
    _robot_wrench_buffers[id].initialize(_config.robot_buffer_size, 6, 1,
                                         "robot_wrench" + std::to_string(id));

    _waypoints_buffers[id].initialize(-1, 7, 1,
                                      "waypoints" + std::to_string(id));
    _eoat_waypoints_buffers[id].initialize(
        -1, 2, 1, "eoat_waypoints" + std::to_string(id));
    _stiffness_buffers[id].initialize(-1, 6, 6,
                                      "stiffness" + std::to_string(id));

    _camera_rgb_timestamp_ms_buffers[id].initialize(
        _config.rgb_buffer_size, 1, 1,
        "camera_rgb" + std::to_string(id) + "_timestamp_ms");
    _pose_timestamp_ms_buffers[id].initialize(
        _config.robot_buffer_size, 1, 1,
        "pose" + std::to_string(id) + "_timestamp_ms");
    _vel_timestamp_ms_buffers[id].initialize(
        _config.robot_buffer_size, 1, 1,  // pose/vel buffers have the same size
        "vel" + std::to_string(id) + "_timestamp_ms");
    _eoat_timestamp_ms_buffers[id].initialize(
        _config.eoat_buffer_size, 1, 1,
        "eoat" + std::to_string(id) + "_timestamp_ms");
    _wrench_timestamp_ms_buffers[id].initialize(
        _config.wrench_buffer_size, 1, 1,
        "wrench" + std::to_string(id) + "_timestamp_ms");
    _robot_wrench_timestamp_ms_buffers[id].initialize(
        _config.robot_buffer_size, 1, 1,
        "robot_wrench" + std::to_string(id) + "_timestamp_ms");
    _waypoints_timestamp_ms_buffers[id].initialize(
        -1, 1, 1, "waypoints" + std::to_string(id) + "_timestamp_ms");
    _eoat_waypoints_timestamp_ms_buffers[id].initialize(
        -1, 1, 1, "eoat_waypoints" + std::to_string(id) + "_timestamp_ms");
    _stiffness_timestamp_ms_buffers[id].initialize(
        -1, 1, 1, "stiffness" + std::to_string(id) + "_timestamp_ms");
  }

  // initialize the buffer mutexes
  for (int id : _id_list) {
    _camera_rgb_buffer_mtxs.emplace_back();
    _pose_buffer_mtxs.emplace_back();
    _vel_buffer_mtxs.emplace_back();
    _eoat_buffer_mtxs.emplace_back();
    _wrench_buffer_mtxs.emplace_back();
    _robot_wrench_buffer_mtxs.emplace_back();
    _waypoints_buffer_mtxs.emplace_back();
    _eoat_waypoints_buffer_mtxs.emplace_back();
    _stiffness_buffer_mtxs.emplace_back();
  }

  // initialize thread status variables
  // indicates the state of each thread
  for (int id : _id_list) {
    _states_robot_thread_ready.push_back(false);
    _states_eoat_thread_ready.push_back(false);
    _states_rgb_thread_ready.push_back(false);
    _states_wrench_thread_ready.push_back(false);
    _states_robot_thread_saving.push_back(false);
    _states_eoat_thread_saving.push_back(false);
    _states_rgb_thread_saving.push_back(false);
    _states_wrench_thread_saving.push_back(false);
    _states_robot_seq_id.push_back(0);
    _states_eoat_seq_id.push_back(0);
    _states_rgb_seq_id.push_back(0);
    _states_wrench_seq_id.push_back(0);
    _states_logging_thread_ready.push_back(false);
  }

  // initialize logging buffers for lock-free logging (10 seconds at 1kHz)
  for (int id : _id_list) {
    RUT::DataBuffer<RobotLogData> buffer;
    buffer.initialize(10000);  // 10000 samples = 10 seconds at 1kHz
    _logging_buffers.push_back(buffer);
    _logging_buffer_mtxs.emplace_back();
    _logging_buffer_overflow.emplace_back(false);
  }

  // initialize additional shared variables
  for (int id : _id_list) {
    _ctrl_rgb_folders.push_back("");
    _ctrl_robot_data_streams.push_back(std::ofstream());
    _ctrl_eoat_data_streams.push_back(std::ofstream());
    _ctrl_wrench_data_streams.push_back(std::ofstream());
    _ctrl_torque_data_streams.push_back(std::ofstream());
    _ctrl_joint_data_streams.push_back(std::ofstream());
    _color_mats.push_back(cv::Mat());
    _color_mat_mtxs.emplace_back();
    _poses_fb.push_back(Eigen::VectorXd());
    _poses_fb_mtxs.emplace_back();
    _wrench_fb.push_back(Eigen::VectorXd());
    _wrench_fb_mtxs.emplace_back();
    _camera_rgb_timestamps_ms.push_back(Eigen::VectorXd());
    _pose_timestamps_ms.push_back(Eigen::VectorXd());
    _vel_timestamps_ms.push_back(Eigen::VectorXd());
    _eoat_timestamps_ms.push_back(Eigen::VectorXd());
    _wrench_timestamps_ms.push_back(Eigen::VectorXd());
    _robot_wrench_timestamps_ms.push_back(Eigen::VectorXd());
  }

  /*
    launch the threads and calls to their respective loops in manip_server_loops.cpp:
      - RGB
      - Wrench
      - Robot
      - EOAT

      thread runs until _ctrl_flag_running is false
  */

  _ctrl_flag_running = true;
  std::cout << "[ManipServer] Starting the threads.\n";
  for (int id : _id_list) {
    if (_config.run_rgb_thread) {
      _rgb_threads.emplace_back(&ManipServer::rgb_loop, this, std::ref(time0),
                                id);
    }
    if (_config.run_robot_thread) {
      if (_config.controller_selection == ControllerSelection::ADMITTANCE_CONTROLLER) {
        _robot_threads.emplace_back(&ManipServer::robot_admittance_loop, this,
                                    std::ref(time0), id);
      } else if (_config.controller_selection == ControllerSelection::IMPEDANCE_CONTROLLER) {
        _robot_threads.emplace_back(&ManipServer::robot_impedance_loop, this,
                                    std::ref(time0), id);
      }
      // Start logging thread for each robot (handles file I/O separately)
      _robot_threads.emplace_back(&ManipServer::robot_logging_loop, this,
                                  std::ref(time0), id);
    }
    if (_config.run_wrench_thread && _config.force_sensing_mode != ForceSensingMode::JOINT_SENSORS) {
      _wrench_threads.emplace_back(&ManipServer::ext_sensor_wrench_loop, this,
                                   std::ref(time0), wrench_publish_rate[id],
                                   id);
    }else if (_config.run_wrench_thread && _config.force_sensing_mode == ForceSensingMode::JOINT_SENSORS) {
      //if using joint torque sensors, launch the joint torque loop instead
      _wrench_threads.emplace_back(&ManipServer::joint_sensor_wrench_loop, this,
                                  std::ref(time0), _config.joint_sensor_frequency, id);
    }
    if (_config.run_eoat_thread) {
      _eoat_threads.emplace_back(&ManipServer::eoat_loop, this, std::ref(time0),
                                 id);
    }
    if (_config.teleop && gamepad_ptrs.size() > id && gamepad_ptrs[id]) {
      _teleop_threads.emplace_back(&ManipServer::teleop_loop, this, std::ref(time0),
                                   id);
    }
  }
  if (_config.plot_rgb) {
    // pause 1s, then start the rgb plot thread
    std::this_thread::sleep_for(std::chrono::seconds(1));
    _rgb_plot_thread = std::thread(&ManipServer::rgb_plot_loop, this);
  }

  // wait for threads to be ready
  std::cout << "[ManipServer] Waiting for threads to be ready.\n";
  while (true) {
    bool all_ready = true;
    {
          std::lock_guard<std::mutex> lock(_ctrl_mtx);
          for (int id : _id_list) {
        if (_config.run_robot_thread && !_states_robot_thread_ready[id]) {
          all_ready = false;
        }
        if (_config.run_wrench_thread && !_states_wrench_thread_ready[id]) {
          all_ready = false;
        }
        if (_config.run_rgb_thread && !_states_rgb_thread_ready[id]) {
          all_ready = false;
        }
      }
      if (_config.plot_rgb) {
        all_ready = all_ready && _state_plot_thread_ready;
      }
    }
    if (all_ready) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::cout << "[ManipServer] All threads are ready." << std::endl;
  std::cout << "[ManipServer] Done initialization." << std::endl;
  return true;
}

void ManipServer::join_threads() {
  std::cout << "[ManipServer]: Waiting for threads to join." << std::endl;
  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_running = false;
  }

  // join the threads
  if (_config.run_rgb_thread) {
    std::cout << "[ManipServer]: Waiting for rgb threads to join." << std::endl;
    for (auto& rgb_thread : _rgb_threads) {
      rgb_thread.join();
    }
  }
  if (_config.run_wrench_thread) {
    std::cout << "[ManipServer]: Waiting for wrench threads to join."
              << std::endl;
    for (auto& wrench_thread : _wrench_threads) {
      wrench_thread.join();
    }
  }
  if (_config.run_robot_thread) {
    std::cout << "[ManipServer]: Waiting for robot threads to join."
              << std::endl;
    for (auto& robot_thread : _robot_threads) {
      robot_thread.join();
    }
  }
  if (_config.run_eoat_thread) {
    std::cout << "[ManipServer]: Waiting for eoat threads to join."
              << std::endl;
    for (auto& eoat_thread : _eoat_threads) {
      eoat_thread.join();
    }
  }
  if (_config.teleop) {
    std::cout << "[ManipServer]: Waiting for teleop threads to join."
              << std::endl;
    for (auto& teleop_thread : _teleop_threads) {
      teleop_thread.join();
    }
    // Cleanup SpaceMouse devices
    for (auto& sm_ptr : spacemouse_ptrs) {
      if (sm_ptr) {
        sm_ptr->cleanup();
      }
    }
    // Cleanup Gamepad devices
    for (auto& gp_ptr : gamepad_ptrs) {
      if (gp_ptr) {
        gp_ptr->cleanup();
      }
    }
  }
  if (_config.plot_rgb) {
    std::cout << "[ManipServer]: Waiting for plotting thread to join."
              << std::endl;
    _rgb_plot_thread.join();
  }

  std::cout << "[ManipServer]: Threads have joined. Exiting." << std::endl;
}

bool ManipServer::is_ready() {
  for (int id : _id_list) {
    if (_config.run_rgb_thread) {
      std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
      if (!_camera_rgb_buffers[id].is_full()) {
        std::cout << id << ": Camera RGB buffer not full: size: "
                  << _camera_rgb_buffers[id].size() << std::endl;
        return false;
      }
    }

    if (_config.run_robot_thread) {
      std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
      if (!_pose_buffers[id].is_full()) {
        std::cout << id << ": Pose buffer not full: size: "
                  << _pose_buffers[id].size() << std::endl;
        return false;
      }
    }

    if (_config.run_eoat_thread) {
      std::lock_guard<std::mutex> lock(_eoat_buffer_mtxs[id]);
      if (!_eoat_buffers[id].is_full()) {
        std::cout << id << ": EoAT buffer not full: size: "
                  << _eoat_buffers[id].size() << std::endl;
        return false;
      }
    }

    if (_config.run_wrench_thread) {
      std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
      if (!_wrench_buffers[id].is_full()) {
        std::cout << id << ": wrench buffer not full: size: "
                  << _wrench_buffers[id].size() << std::endl;
        return false;
      }
    }
  }
  return true;
}

bool ManipServer::is_running() {
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  return _ctrl_flag_running;
}

const Eigen::MatrixXd ManipServer::get_camera_rgb(int k, int id) {
  std::lock_guard<std::mutex> lock(_camera_rgb_buffer_mtxs[id]);
  _camera_rgb_timestamps_ms[id] =
      _camera_rgb_timestamp_ms_buffers[id].get_last_k(k);
  return _camera_rgb_buffers[id].get_last_k(k);
}

const Eigen::MatrixXd ManipServer::get_wrench(int k, int id) {
  std::lock_guard<std::mutex> lock(_wrench_buffer_mtxs[id]);
  _wrench_timestamps_ms[id] = _wrench_timestamp_ms_buffers[id].get_last_k(k);
  auto wrench_data = _wrench_buffers[id].get_last_k(k);

  return wrench_data;
}

const Eigen::MatrixXd ManipServer::get_robot_wrench(int k, int id) {
  std::lock_guard<std::mutex> lock(_robot_wrench_buffer_mtxs[id]);
  _robot_wrench_timestamps_ms[id] =
      _robot_wrench_timestamp_ms_buffers[id].get_last_k(k);
  return _robot_wrench_buffers[id].get_last_k(k);
}

const Eigen::MatrixXd ManipServer::get_pose(int k, int id) {
  std::lock_guard<std::mutex> lock(_pose_buffer_mtxs[id]);
  _pose_timestamps_ms[id] = _pose_timestamp_ms_buffers[id].get_last_k(k);
  return _pose_buffers[id].get_last_k(k);
}

const Eigen::MatrixXd ManipServer::get_vel(int k, int id) {
  std::lock_guard<std::mutex> lock(_vel_buffer_mtxs[id]);
  _vel_timestamps_ms[id] = _vel_timestamp_ms_buffers[id].get_last_k(k);
  return _vel_buffers[id].get_last_k(k);
}

const int ManipServer::get_test() {
  _test_timestamp_ms = _timer.toc_ms();
  return 0;
}

const Eigen::VectorXd ManipServer::get_camera_rgb_timestamps_ms(int id) {
  return _camera_rgb_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_wrench_timestamps_ms(int id) {
  return _wrench_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_robot_wrench_timestamps_ms(int id) {
  return _robot_wrench_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_pose_timestamps_ms(int id) {
  return _pose_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_vel_timestamps_ms(int id) {
  return _vel_timestamps_ms[id];
}
const Eigen::VectorXd ManipServer::get_eoat_timestamps_ms(int id) {
  return _eoat_timestamps_ms[id];
}

const double ManipServer::get_test_timestamp_ms() {
  return _test_timestamp_ms;
}

double ManipServer::get_timestamp_now_ms() {
  return _timer.toc_ms();
}

void ManipServer::set_high_level_maintain_position() {
  if (!_config.run_robot_thread) {
    return;
  }
  // clear existing targets
  clear_cmd_buffer();

  RUT::Vector7d pose_fb;
  for (int id : _id_list) {
    // get the current pose as the only new target
    if (!_config.mock_hardware) {
      robot_ptrs[id]->getCartesian(
          pose_fb);  // use the current pose as the reference
    }
    set_target_pose(pose_fb, 200, id);
    set_target_pose(pose_fb, 1000, id);
  }
  // wait for > 100ms before turn on high stiffness
  // So that the internal target in the interpolation controller gets refreshed
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
    // set the robot to have high stiffness, but still compliant
    if (_config.controller_selection == ControllerSelection::ADMITTANCE_CONTROLLER) {
      _admittance_controllers[id].setStiffnessMatrix(_stiffnesses_high[id]);
      _admittance_controllers[id].setDampingMatrix(_dampings_high[id]);
    } else if (_config.controller_selection == ControllerSelection::IMPEDANCE_CONTROLLER) {
      _impedance_controllers[id].setStiffnessMatrix(_stiffnesses_high[id]);
      _impedance_controllers[id].setDampingMatrix(_dampings_high[id]);
    }
  }
}

void ManipServer::set_high_level_free_jogging() {
  if (!_config.run_robot_thread) {
    return;
  }
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_controller_mtxs[id]);
    // set the robot to be compliant
    if (_config.controller_selection == ControllerSelection::ADMITTANCE_CONTROLLER) {
      _admittance_controllers[id].setStiffnessMatrix(_stiffnesses_low[id]);
      _admittance_controllers[id].setDampingMatrix(_dampings_low[id]);
    } else if (_config.controller_selection == ControllerSelection::IMPEDANCE_CONTROLLER) {
      _impedance_controllers[id].setStiffnessMatrix(_stiffnesses_low[id]);
      _impedance_controllers[id].setDampingMatrix(_dampings_low[id]);
    }
  }
}

void ManipServer::set_target_pose(const Eigen::Ref<RUT::Vector7d> pose,
                                  double dt_in_future_ms, int robot_id) {
  std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[robot_id]);
  _waypoints_buffers[robot_id].put(pose);
  _waypoints_timestamp_ms_buffers[robot_id].put(
      _timer.toc_ms() + dt_in_future_ms);  // 1s in the future
}

void ManipServer::set_force_controlled_axis(const RUT::Matrix6d& Tr, int n_af,
                                            int robot_id) {
  std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
  if (_config.controller_selection == ControllerSelection::ADMITTANCE_CONTROLLER) {
    _admittance_controllers[robot_id].setForceControlledAxis(Tr, n_af);
  } else if (_config.controller_selection == ControllerSelection::IMPEDANCE_CONTROLLER) {
    _impedance_controllers[robot_id].setForceControlledAxis(Tr, n_af);
  }
}

void ManipServer::set_stiffness_matrix(const RUT::Matrix6d& stiffness,
                                       int robot_id) {
  std::lock_guard<std::mutex> lock(_controller_mtxs[robot_id]);
  if (_config.controller_selection == ControllerSelection::ADMITTANCE_CONTROLLER) {
    _admittance_controllers[robot_id].setStiffnessMatrix(stiffness);
  } else if (_config.controller_selection == ControllerSelection::IMPEDANCE_CONTROLLER) {
    _impedance_controllers[robot_id].setStiffnessMatrix(stiffness);
  }
}

void ManipServer::clear_cmd_buffer() {
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[id]);
    _waypoints_buffers[id].clear();
    _waypoints_timestamp_ms_buffers[id].clear();
  }
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_eoat_waypoints_buffer_mtxs[id]);
    _eoat_waypoints_buffers[id].clear();
    _eoat_waypoints_timestamp_ms_buffers[id].clear();
  }
  for (int id : _id_list) {
    std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[id]);
    _stiffness_buffers[id].clear();
    _stiffness_timestamp_ms_buffers[id].clear();
  }
}

/*
  1. Points in _waypoints_buffer are not yet scheduled to be executed. 
  2. interpolation_controller will take the oldest N points away from _waypoints_buffer and _waypoints_timestamp_ms_buffer 
    and interpolate them to generate a trajectory.
  3. schedule_waypoints adds timed waypoints to the buffer following the procedures below:
    a. remove input waypoints that are in the past.
    b. remove existing waypoints that are newer than input waypoints.
    c. Adds remaining of a to the end of b.
*/
// #define DEBUG_WP_SCHEDULING
void ManipServer::schedule_waypoints(const Eigen::MatrixXd& waypoints,
                                     const Eigen::VectorXd& timepoints_ms,
                                     int robot_id) {
  double curr_time = _timer.toc_ms();
  // check the shape of inputs
  if (waypoints.rows() != 7) {
    std::cerr << "[ManipServer][schedule_waypoints] Waypoints should have 7 "
                 "rows. Exiting."
              << std::endl;
    return;
  }
  if (timepoints_ms.size() != waypoints.cols()) {
    std::cerr << "[ManipServer][schedule_waypoints] Waypoints and "
                 "timepoints_ms should have the same "
                 "number of columns. Exiting."
              << std::endl;
    return;
  }

#ifdef DEBUG_WP_SCHEDULING
  std::cout << "[ManipServer][schedule_waypoints] waypoints: \n"
            << waypoints << std::endl;
  std::cout << "[ManipServer][schedule_waypoints] timepoints_ms: \n"
            << timepoints_ms.transpose() << std::endl;
  std::cout << "[ManipServer][schedule_waypoints] curr_time: " << curr_time
            << std::endl;
#endif
  /*
   * a. Get rid of input waypoints that are in the past
   */
  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    // all input points are in the past. Do nothing.
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_waypoints_buffer_mtxs[robot_id]);
    /*
   * b. Get rid of existing waypoints that are newer than input waypoints
   */
    int existing_id_end = 0;
    for (int i = 0; i < _waypoints_timestamp_ms_buffers[robot_id].size(); i++) {
      if (_waypoints_timestamp_ms_buffers[robot_id][i] >
          timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _waypoints_buffers[robot_id].remove_last_k(
        _waypoints_buffers[robot_id].size() - existing_id_end);
    _waypoints_timestamp_ms_buffers[robot_id].remove_last_k(
        _waypoints_timestamp_ms_buffers[robot_id].size() - existing_id_end);
    assert(_waypoints_buffers[robot_id].size() ==
           _waypoints_timestamp_ms_buffers[robot_id].size());

    /*
   * c. Add remaining of a to the end of b
   */
    int input_id_end = timepoints_ms.size();
#ifdef DEBUG_WP_SCHEDULING
    std::cout << "[ManipServer][schedule_waypoints] input_id_start: "
              << input_id_start << std::endl;
    std::cout << "[ManipServer][schedule_waypoints] input_id_end: "
              << input_id_end << std::endl;
#endif
    for (int i = input_id_start; i < input_id_end; i++) {
#ifdef DEBUG_WP_SCHEDULING
      std::cout << "[ManipServer][schedule_waypoints] Adding waypoint: "
                << waypoints.col(i).transpose()
                << " at time: " << timepoints_ms(i) << std::endl;
#endif
      _waypoints_buffers[robot_id].put(waypoints.col(i));
      _waypoints_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
    }
  }
}  // end function schedule_waypoints

void ManipServer::schedule_eoat_waypoints(const Eigen::MatrixXd& eoat_waypoints,
                                          const Eigen::VectorXd& timepoints_ms,
                                          int robot_id) {
  double curr_time = _timer.toc_ms();
  // check the shape of inputs
  if (eoat_waypoints.rows() != 2) {
    std::cerr
        << "[ManipServer][schedule_eoat_waypoints] Waypoints should have 2 "
           "rows. Exiting."
        << std::endl;
    return;
  }
  if (timepoints_ms.size() != eoat_waypoints.cols()) {
    std::cerr << "[ManipServer][schedule_eoat_waypoints] Waypoints and "
                 "timepoints_ms should have the same "
                 "number of columns. Exiting."
              << std::endl;
    return;
  }

#ifdef DEBUG_EOAT_WP_SCHEDULING
  std::cout << "[ManipServer][schedule_eoat_waypoints] eoat_waypoints: \n"
            << eoat_waypoints << std::endl;
  std::cout << "[ManipServer][schedule_eoat_waypoints] timepoints_ms: \n"
            << timepoints_ms.transpose() << std::endl;
  std::cout << "[ManipServer][schedule_eoat_waypoints] curr_time: " << curr_time
            << std::endl;
#endif
  /*
   * a. Get rid of input eoat_waypoints that are in the past
   */
  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    // all input points are in the past. Do nothing.
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_eoat_waypoints_buffer_mtxs[robot_id]);
    /*
   * b. Get rid of existing eoat_waypoints that are newer than input eoat_waypoints
   */
    int existing_id_end = 0;
    for (int i = 0; i < _eoat_waypoints_timestamp_ms_buffers[robot_id].size();
         i++) {
      if (_eoat_waypoints_timestamp_ms_buffers[robot_id][i] >
          timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _eoat_waypoints_buffers[robot_id].remove_last_k(
        _eoat_waypoints_buffers[robot_id].size() - existing_id_end);
    _eoat_waypoints_timestamp_ms_buffers[robot_id].remove_last_k(
        _eoat_waypoints_timestamp_ms_buffers[robot_id].size() -
        existing_id_end);
    assert(_eoat_waypoints_buffers[robot_id].size() ==
           _eoat_waypoints_timestamp_ms_buffers[robot_id].size());

    /*
   * c. Add remaining of a to the end of b
   */
    int input_id_end = timepoints_ms.size();
#ifdef DEBUG_EOAT_WP_SCHEDULING
    std::cout << "[ManipServer][schedule_eoat_waypoints] input_id_start: "
              << input_id_start << std::endl;
    std::cout << "[ManipServer][schedule_eoat_waypoints] input_id_end: "
              << input_id_end << std::endl;
#endif
    for (int i = input_id_start; i < input_id_end; i++) {
#ifdef DEBUG_EOAT_WP_SCHEDULING
      std::cout << "[ManipServer][schedule_eoat_waypoints] Adding waypoint: "
                << eoat_waypoints.col(i).transpose()
                << " at time: " << timepoints_ms(i) << std::endl;
#endif
      _eoat_waypoints_buffers[robot_id].put(eoat_waypoints.col(i));
      _eoat_waypoints_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
    }
  }
}  // end function schedule_waypoints

/*
  1. Points in _waypoints_buffer are not yet scheduled to be executed. 
  2. interpolation_controller will take the oldest N points away from _waypoints_buffer and _waypoints_timestamp_ms_buffer 
    and interpolate them to generate a trajectory.
  3. schedule_waypoints adds timed waypoints to the buffer following the procedures below:
    a. remove input waypoints that are in the past.
    b. remove existing waypoints that are newer than input waypoints.
    c. Adds remaining of a to the end of b.
*/
// #define DEBUG_STIFFNESS_SCHEDULING
void ManipServer::schedule_stiffness(const Eigen::MatrixXd& stiffnesses,
                                     const Eigen::VectorXd& timepoints_ms,
                                     int robot_id) {
  double curr_time = _timer.toc_ms();
  // check the shape of inputs
  if (stiffnesses.rows() != 6) {
    std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have 6 "
                 "rows. Exiting."
              << std::endl;
    return;
  }
  if (stiffnesses.cols() / timepoints_ms.size() != 6) {
    std::cerr << "[ManipServer][schedule_stiffness] stiffnesses should have "
                 "6x number of columns as timepoints_ms. Exiting."
              << std::endl;
    return;
  }

#ifdef DEBUG_STIFFNESS_SCHEDULING
  std::cout << "[ManipServer][schedule_stiffness] stiffnesses: \n"
            << stiffnesses << std::endl;
  std::cout << "[ManipServer][schedule_stiffness] timepoints_ms: \n"
            << timepoints_ms.transpose() << std::endl;
  std::cout << "[ManipServer][schedule_stiffness] curr_time: " << curr_time
            << std::endl;
#endif
  /*
   * a. Get rid of inputs that are in the past
   */
  int input_id_start = 0;
  for (int i = 0; i < timepoints_ms.size(); i++) {
    if (timepoints_ms(i) > curr_time) {
      input_id_start = i;
      break;
    }
  }
  if (input_id_start >= timepoints_ms.size()) {
    // all input points are in the past. Do nothing.
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_stiffness_buffer_mtxs[robot_id]);
    /*
     * b. Get rid of existing stiffness that are newer than input stiffness
     */
    int existing_id_end = 0;
    for (int i = 0; i < _stiffness_timestamp_ms_buffers[robot_id].size(); i++) {
      if (_stiffness_timestamp_ms_buffers[robot_id][i] >
          timepoints_ms(input_id_start)) {
        existing_id_end = i;
        break;
      }
    }
    _stiffness_buffers[robot_id].remove_last_k(
        _stiffness_buffers[robot_id].size() - existing_id_end);
    _stiffness_timestamp_ms_buffers[robot_id].remove_last_k(
        _stiffness_timestamp_ms_buffers[robot_id].size() - existing_id_end);
    assert(_stiffness_buffers[robot_id].size() ==
           _stiffness_timestamp_ms_buffers[robot_id].size());
    /*
     * c. Add remaining of a to the end of b
     */
    int input_id_end = timepoints_ms.size();
#ifdef DEBUG_STIFFNESS_SCHEDULING
    std::cout << "[ManipServer][schedule_stiffness] input_id_start: "
              << input_id_start << std::endl;
    std::cout << "[ManipServer][schedule_stiffness] input_id_end: "
              << input_id_end << std::endl;
#endif
    for (int i = input_id_start; i < input_id_end; i++) {
#ifdef DEBUG_STIFFNESS_SCHEDULING
      std::cout << "[ManipServer][schedule_stiffness] Adding stiffness:\n"
                << stiffnesses.middleCols<6>(6 * i)
                << " at time: " << timepoints_ms(i) << std::endl;
#endif
      _stiffness_buffers[robot_id].put(stiffnesses.middleCols<6>(i * 6));
      _stiffness_timestamp_ms_buffers[robot_id].put(timepoints_ms(i));
    }
  }
}  // end function schedule_stiffness

void ManipServer::start_saving_data_for_a_new_episode() {
  // create episode folders
  std::vector<std::string> robot_json_file_names;
  std::vector<std::string> wrench_json_file_names;
  std::vector<std::string> torque_json_file_names;
  std::vector<std::string> joint_json_file_names;
  create_folder_for_new_episode(_config.data_folder, _id_list,
                                _ctrl_rgb_folders, robot_json_file_names,
                                wrench_json_file_names, torque_json_file_names,
                                joint_json_file_names);

  std::cout << "[main] New episode. rgb_folder_name: " << _ctrl_rgb_folders[0]
            << std::endl;

  // get rgb folder and low dim json file for saving data
  for (int id : _id_list) {
    _ctrl_robot_data_streams[id].open(robot_json_file_names[id]);
    _ctrl_wrench_data_streams[id].open(wrench_json_file_names[id]);
    _ctrl_torque_data_streams[id].open(torque_json_file_names[id]);
    _ctrl_joint_data_streams[id].open(joint_json_file_names[id]);
  }

  {
    std::lock_guard<std::mutex> lock(_ctrl_mtx);
    _ctrl_flag_saving = true;
  }
}

void ManipServer::stop_saving_data() {
  std::lock_guard<std::mutex> lock(_ctrl_mtx);
  _ctrl_flag_saving = false;
}

bool ManipServer::is_saving_data() {
  bool is_saving = false;
  for (int id : _id_list) {
    is_saving = is_saving || _states_robot_thread_saving[id] ||
                _states_rgb_thread_saving[id] ||
                _states_wrench_thread_saving[id];
  }
  return is_saving;
}
