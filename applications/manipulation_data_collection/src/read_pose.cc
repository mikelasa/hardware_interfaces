/*
 * Read Franka EE pose directly via libfranka — no ManipServer required.
 *
 * Run this utility standalone (no other program may hold the robot connection).
 * Put the robot in gravity-compensation mode with the blue button on the base,
 * guide it to each desired pose, then press Enter to capture.
 * At the end the WAYPOINT_POSES array is printed ready to paste into
 * main_scripted_motion.cc.
 *
 * Usage:  ./read_pose [robot_ip]   (default: 172.17.6.164)
 */

#include <array>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <franka/robot.h>

static const int         N_CAPTURES = 18;
static const std::string DEFAULT_IP = "172.17.6.164";

int main(int argc, char** argv) {
  const std::string ip = (argc > 1) ? argv[1] : DEFAULT_IP;

  std::cout << "[read_pose] Connecting to Franka at " << ip << " ...\n";
  franka::Robot robot(ip);
  std::cout << "[read_pose] Connected.\n\n"
            << "[read_pose] Press the BLUE BUTTON on the Panda base to enable\n"
            << "[read_pose] gravity compensation, then guide the arm by hand.\n"
            << "[read_pose] Press Enter to capture each pose (" << N_CAPTURES << " total).\n\n";

  std::vector<std::array<double, 7>> captured;    // EE poses
  std::vector<std::array<double, 7>> q_captured;  // joint angles

  std::atomic<bool> enter_pressed{false};
  std::atomic<bool> done{false};
  std::thread input_thread([&]() {
    std::string line;
    while (!done && std::getline(std::cin, line))
      enter_pressed = true;
  });

  while ((int)captured.size() < N_CAPTURES) {
    franka::RobotState state = robot.readOnce();

    // O_T_EE: 4×4 column-major homogeneous transform (base → EE)
    const auto& ee = state.O_T_EE;
    double x = ee[12], y = ee[13], z = ee[14];

    // Rotation sub-matrix → quaternion [qw, qx, qy, qz]
    Eigen::Matrix3d R;
    R << ee[0], ee[4], ee[8],
         ee[1], ee[5], ee[9],
         ee[2], ee[6], ee[10];
    Eigen::Quaterniond q(R);

    if (enter_pressed) {
      enter_pressed = false;
      captured.push_back({x, y, z, q.w(), q.x(), q.y(), q.z()});
      q_captured.push_back(state.q);
      std::cout << "\n[read_pose] Captured " << captured.size() << "/" << N_CAPTURES
                << "\n  EE  : "
                << std::fixed << std::setprecision(4)
                << x << "  " << y << "  " << z << "    "
                << q.w() << "  " << q.x() << "  " << q.y() << "  " << q.z()
                << "\n  q   : ";
      for (int i = 0; i < 7; ++i)
        std::cout << std::setprecision(4) << state.q[i] << (i < 6 ? "  " : "\n");
      if ((int)captured.size() < N_CAPTURES)
        std::cout << "[read_pose] Move to next pose and press Enter...\n";
    } else {
      std::cout << "\r  q: ";
      for (int i = 0; i < 7; ++i)
        std::cout << std::fixed << std::setprecision(4) << state.q[i] << (i < 6 ? "  " : "  ");
      std::cout << std::flush;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  done = true;
  input_thread.detach();

  // ── Print joint angles ready to paste into wrench_calib_joint_recorder.cc ─
  std::cout << "\n\n"
            << "// ── paste into wrench_calib_joint_recorder.cc ──────────────────────────\n"
            << "static constexpr int N_REF = " << N_CAPTURES << ";\n"
            << "static constexpr double Q_REF[N_REF][7] = {\n"
            << "    //   q1       q2       q3       q4       q5       q6       q7\n";
  for (int i = 0; i < (int)q_captured.size(); ++i) {
    const auto& q = q_captured[i];
    std::cout << std::fixed << std::setprecision(4)
              << "    { " << q[0] << ",  " << q[1] << ",  " << q[2] << ",  "
              << q[3] << ",  " << q[4] << ",  " << q[5] << ",  " << q[6] << "}";
    if (i < (int)q_captured.size() - 1) std::cout << ",";
    std::cout << "\n";
  }
  std::cout << "};\n"
            << "// ─────────────────────────────────────────────────────────────────────────\n";

  return 0;
}