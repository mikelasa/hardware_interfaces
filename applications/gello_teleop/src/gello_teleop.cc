#include <franka/franka.h>
#include <gello/gello.h>
#include <force_control_impedance/impedance_controller.h>
#include <force_control_impedance/config_deserialize_impedance.h>
#include <RobotUtilities/timer_linux.h>
#include <RobotUtilities/spatial_utilities.h>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <csignal>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <thread>
#include <chrono>
#include <future>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static double max_joint_error(const RUT::VectorXd& a, const std::vector<double>& b) {
  double err = 0.0;
  for (int i = 0; i < 7; ++i)
    err = std::max(err, std::abs(a[i] - b[i]));
  return err;
}

// 4×4 column-major transform array → [x, y, z, qw, qx, qy, qz] (metres)
static RUT::Vector7d T16_to_pose(const std::array<double, 16>& T) {
  Eigen::Matrix4d M;
  for (int c = 0; c < 4; ++c)
    for (int r = 0; r < 4; ++r)
      M(r, c) = T[c * 4 + r];
  Eigen::Affine3d tf(M);
  Eigen::Vector3d   p = tf.translation();
  Eigen::Quaterniond q(tf.rotation());
  RUT::Vector7d pose;
  pose << p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z();
  return pose;
}

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);

  // ── Config ────────────────────────────────────────────────────────────────
  const std::string config_path =
      (argc > 1) ? argv[1]
                 : "/home/robotlab/ACP/hardware_interfaces/applications/"
                   "gello_teleop/config/gello_teleop.yaml";

  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(config_path);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load config: " << e.what() << "\n";
    return 1;
  }

  FRANKA::FRANKAConfig franka_config;
  franka_config.deserialize(cfg["franka"]);

  GelloInterface::GelloConfig gello_config;
  gello_config.deserialize(cfg["gello"]);

  ImpedanceController::ImpedanceControllerConfig impedance_config;
  if (!deserialize(cfg["impedance_controller0"], impedance_config)) {
    std::cerr << "Failed to load impedance controller config\n";
    return 1;
  }

  const double loop_rate_hz =
      cfg["teleop"]["loop_rate_hz"] ? cfg["teleop"]["loop_rate_hz"].as<double>() : 1000.0;
  const double align_threshold =
      cfg["teleop"]["alignment_threshold_rad"]
          ? cfg["teleop"]["alignment_threshold_rad"].as<double>() : 0.15;
  const double lp_alpha =
      cfg["teleop"]["low_pass_alpha"] ? cfg["teleop"]["low_pass_alpha"].as<double>() : 0.1;

  std::vector<double> home_joints = {0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0};
  if (cfg["teleop"]["home_joints"])
    home_joints = cfg["teleop"]["home_joints"].as<std::vector<double>>();

  // ── Init GELLO ────────────────────────────────────────────────────────────
  GelloInterface gello;
  if (!gello.init(gello_config)) {
    std::cerr << "Failed to initialize GELLO on " << gello_config.port << "\n";
    return 1;
  }

  // ── Init Franka ───────────────────────────────────────────────────────────
  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();

  FRANKA robot;
  if (!robot.init(time0, franka_config)) {
    std::cerr << "Failed to initialize Franka\n";
    gello.cleanup();
    return 1;
  }

  // ── Load model and capture initial state ─────────────────────────────────
  franka::Model model(robot.loadModel());
  franka::RobotState rs = robot.getRobotState();
  const std::array<double, 16> F_T_EE = rs.F_T_EE;
  const std::array<double, 16> EE_T_K = rs.EE_T_K;

  RUT::VectorXd q_robot(7);
  if (!robot.getJoints(q_robot)) {
    std::cerr << "Failed to read robot joints\n";
    gello.cleanup();
    return 1;
  }
  RUT::Vector7d pose_robot;
  if (!robot.getCurrentPose(pose_robot)) {
    std::cerr << "Failed to read robot Cartesian pose\n";
    gello.cleanup();
    return 1;
  }

  std::cout << "Robot joints: [";
  for (int i = 0; i < 7; ++i)
    std::cout << std::fixed << std::setprecision(4) << q_robot[i] << (i < 6 ? ", " : "]\n");
  std::cout << "Robot pose (m): x=" << std::setprecision(4) << pose_robot[0]
            << "  y=" << pose_robot[1] << "  z=" << pose_robot[2] << "\n";

  // ── Init impedance controller ─────────────────────────────────────────────
  ImpedanceController impedance_ctrl;
  if (!impedance_ctrl.init(time0, impedance_config, pose_robot)) {
    std::cerr << "Failed to initialize impedance controller\n";
    gello.cleanup();
    return 1;
  }
  // All 6 DOFs are position-controlled (no force axes)
  impedance_ctrl.setForceControlledAxis(RUT::Matrix6d::Identity(), 0);

  // ── 2π offset correction ──────────────────────────────────────────────────
  {
    GelloData gd;
    for (int attempt = 0; attempt < 100; ++attempt) {
      gello.get_data(gd);
      if (!gd.joint_positions.empty()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "GELLO joints (before correction): [";
    for (int i = 0; i < 7; ++i)
      std::cout << std::fixed << std::setprecision(4) << gd.joint_positions[i] << (i < 6 ? ", " : "]\n");

    std::cout << "2π correction (k per joint):      [";
    for (int i = 0; i < 7; ++i) {
      double err = gd.joint_positions[i] - home_joints[i];
      double k   = std::round(err / (2.0 * M_PI));
      gello_config.joint_offsets[i] += k * 2.0 * M_PI * gello_config.joint_signs[i];
      std::cout << std::setw(2) << (int)k << (i < 6 ? ", " : "]\n");
    }

    gello.cleanup();
    if (!gello.init(gello_config)) {
      std::cerr << "Failed to re-init GELLO after offset correction\n";
      return 1;
    }

    for (int attempt = 0; attempt < 200; ++attempt) {
      gello.get_data(gd);
      if (gd.timestamp > 0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "GELLO joints (after  correction): [";
    for (int i = 0; i < 7; ++i)
      std::cout << std::fixed << std::setprecision(4) << gd.joint_positions[i] << (i < 6 ? ", " : "]\n");
    std::cout << "Residual vs home (should be <π):  [";
    for (int i = 0; i < 7; ++i)
      std::cout << std::fixed << std::setprecision(4) << (gd.joint_positions[i] - home_joints[i])
                << (i < 6 ? ", " : "]\n");
  }

  // ── Alignment phase ───────────────────────────────────────────────────────
  std::cout << "\nTarget robot joints (rad):\n";
  for (int i = 0; i < 7; ++i)
    std::cout << "  J" << (i + 1) << ": " << std::fixed << std::setprecision(4) << q_robot[i] << "\n";
  std::cout << "\nMove GELLO to match robot joints. Threshold: " << align_threshold << " rad.\n";
  std::cout << "     J1       J2       J3       J4       J5       J6       J7     | MAX\n";

  bool aligned       = false;
  int  print_counter = 0;
  const RUT::Vector6d zero_wrench = RUT::Vector6d::Zero();
  RUT::Vector7d pose_ref_filtered = pose_robot;  // low-pass state, seeded at current pose

  timer.set_loop_rate_hz(loop_rate_hz);
  timer.tic();

  try {
    while (g_running) {
      timer.sleep_till_next();

      // Update robot state for impedance controller
      RUT::Vector7d pose_fb;
      RUT::Vector6d wrench_fb;
      robot.getCurrentPose(pose_fb);
      robot.getCurrentWrenchTool(wrench_fb);
      franka::RobotState state = robot.getRobotState();

      auto jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
      impedance_ctrl.getJacobian(
          Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data()));
      impedance_ctrl.getRobotState(state);

      GelloData gd;
      gello.get_data(gd);

      RUT::Vector7d pose_ref;

      if (!aligned) {
        // Hold initial Cartesian pose
        pose_ref = pose_robot;

        double err = max_joint_error(q_robot, gd.joint_positions);
        if (++print_counter % 100 == 0) {
          std::cout << "\r";
          for (int i = 0; i < 7; ++i) {
            double e = gd.joint_positions[i] - q_robot[i];
            std::cout << std::fixed << std::setprecision(3) << std::setw(7) << e << "  ";
          }
          std::cout << "| " << std::setw(5) << err << "  " << std::flush;
        }
        if (err < align_threshold) {
          aligned = true;
          std::cout << "\n\nAligned! Teleoperation active. Press Ctrl+C to stop.\n";
        }
      } else {
        // Teleop: FK(GELLO joints) → Cartesian reference
        std::array<double, 7> q_arr{};
        for (int i = 0; i < 7; ++i) q_arr[i] = gd.joint_positions[i];
        auto T = model.pose(franka::Frame::kEndEffector, q_arr, F_T_EE, EE_T_K);
        pose_ref = T16_to_pose(T);
      }

      // Low-pass filter: position lerp, orientation slerp
      {
        Eigen::Quaterniond q_new(pose_ref[3], pose_ref[4], pose_ref[5], pose_ref[6]);
        Eigen::Quaterniond q_filt(pose_ref_filtered[3], pose_ref_filtered[4],
                                  pose_ref_filtered[5], pose_ref_filtered[6]);
        if (q_filt.dot(q_new) < 0.0) q_new.coeffs() = -q_new.coeffs();
        pose_ref_filtered.head<3>() =
            (1.0 - lp_alpha) * pose_ref_filtered.head<3>() + lp_alpha * pose_ref.head<3>();
        Eigen::Quaterniond q_out = q_filt.slerp(lp_alpha, q_new);
        pose_ref_filtered[3] = q_out.w(); pose_ref_filtered[4] = q_out.x();
        pose_ref_filtered[5] = q_out.y(); pose_ref_filtered[6] = q_out.z();
      }

      impedance_ctrl.setRobotStatus(pose_fb, wrench_fb);
      impedance_ctrl.setRobotReference(pose_ref_filtered, zero_wrench);

      RUT::Vector7d torques;
      impedance_ctrl.step(torques);

      if (!robot.setTorques(torques)) {
        std::cerr << "\nsetTorques failed\n";
        break;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "\nException: " << e.what() << "\n";
  }

  std::cout << "\nStopping.\n";
  impedance_ctrl.flushLog();
  {
    auto f = std::async(std::launch::async,
        [&]() { try { robot.finishCurrentMotion(); } catch (...) {} });
    if (f.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
      std::cerr << "Warning: finishCurrentMotion timed out, forcing exit.\n";
  }
  gello.cleanup();
  return 0;
}