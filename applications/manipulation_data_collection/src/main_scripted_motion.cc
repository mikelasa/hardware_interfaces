/*
 * Scripted motion data collection.
 *
 * Moves the robot through 5 Cartesian waypoints defined as offsets from the
 * home pose (the pose at startup), recording force, RGB, and proprioception
 * data for the whole trajectory in one episode.
 *
 * Adjust WAYPOINT_OFFSETS and DT_PER_WAYPOINT_MS before running.
 */

#include <iostream>
#include <thread>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include <table_top_manip/manip_server.h>

// ── Trajectory parameters ─────────────────────────────────────────────────────
// Absolute Cartesian poses in the robot base frame: [x, y, z, qw, qx, qy, qz]
// Units: metres and unit quaternion.
// Tip: run the script once with the robot at the desired positions and read
// the logged "Current pose:" lines to fill these in.
static const int N_WAYPOINTS = 20;
static const double WAYPOINT_POSES[N_WAYPOINTS][7] = {
    //   x        y        z       qw      qx      qy      qz
    { 0.4950,  -0.0024,  0.4060,  0.1416,  0.3780,  0.9133,  0.0537},
    { 0.5316,  -0.0093,  0.2594,  0.1850,  0.3812,  0.9026,  0.0763},
    { 0.6242,  -0.0314,  0.1800,  0.2825,  0.3788,  0.8753,  0.1029},
    { 0.7316,  -0.0386,  0.1709,  0.3258,  0.3744,  0.8603,  0.1168},
    { 0.6098,  -0.0388,  0.2866,  0.2415,  0.3697,  0.8939,  0.0770},
    { 0.6884,  -0.0412,  0.2832,  0.2864,  0.3680,  0.8797,  0.0928},
    { 0.7631,  -0.0412,  0.2875,  0.2226,  0.3919,  0.8898,  0.0714},
    { 0.8261,  -0.0373,  0.3141,  0.2200,  0.3883,  0.8923,  0.0681},
    { 0.5769,  0.0616,  0.2113,  0.2812,  0.2896,  0.9046,  0.1370},
    { 0.6690,  -0.0262,  0.2535,  0.2964,  0.3825,  0.8707,  0.0878},
    { 0.6158,  0.1033,  0.1874,  0.2913,  0.3172,  0.8861,  0.1716},
    { 0.7145,  0.1134,  0.1640,  0.3613,  0.2953,  0.8614,  0.2007},
    { 0.7706,  0.0596,  0.1628,  0.4105,  0.3222,  0.8338,  0.1804},
    { 0.7215,  0.0680,  0.2453,  0.3233,  0.3014,  0.8875,  0.1303},
    { 0.6920,  0.1502,  0.2596,  0.2254,  0.2813,  0.9237,  0.1294},
    { 0.7859,  0.1693,  0.2751,  0.2305,  0.2609,  0.9302,  0.1158},
    { 0.6034,  0.0853,  0.2841,  0.2272,  0.2653,  0.9329,  0.0879},
    { 0.5643,  0.0657,  0.1733,  0.3469,  0.2718,  0.8892,  0.1226},
    { 0.6148,  0.0770,  0.2579,  0.2715,  0.2870,  0.9129,  0.1026},
    { 0.7120,  0.0998,  0.3003,  0.1756,  0.2889,  0.9393,  0.0585},};
    /*
    { 0.7952,  0.1453,  0.2962,  0.2117,  0.2629,  0.9384,  0.0741},
    { 0.6593,  0.1339,  0.1946,  0.3053,  0.2654,  0.9040,  0.1381},
    { 0.7097,  0.0698,  0.1753,  0.3542,  0.2923,  0.8830,  0.0975},
    { 0.7508,  0.1177,  0.2749,  0.2349,  0.3114,  0.9179,  0.0724},
    { 0.8045,  0.0730,  0.2391,  0.3303,  0.3140,  0.8872,  0.0720},
    { 0.5381,  -0.1118,  0.1921,  0.3167,  0.4545,  0.8314,  0.0431},
    { 0.6187,  -0.1234,  0.1693,  0.3645,  0.4310,  0.8243,  0.0425},
    { 0.6971,  -0.1169,  0.1664,  0.3940,  0.4066,  0.8224,  0.0553},
    { 0.7824,  -0.1237,  0.1520,  0.4220,  0.3944,  0.8137,  0.0645},
    { 0.5700,  -0.1690,  0.1831,  0.3566,  0.4477,  0.8200,  0.0068},
    { 0.6643,  -0.1655,  0.1663,  0.4055,  0.4093,  0.8171,  0.0180},
    { 0.7451,  -0.1668,  0.1500,  0.4262,  0.4124,  0.8042,  0.0379},
    { 0.7753,  -0.2262,  0.1619,  0.4327,  0.4147,  0.8005,  -0.0045},
    { 0.6635,  -0.2142,  0.1715,  0.3796,  0.4389,  0.8144,  -0.0039},
    { 0.5555,  -0.1949,  0.1743,  0.3526,  0.4612,  0.8142,  -0.0040},
    { 0.5258,  -0.0939,  0.2502,  0.2891,  0.4072,  0.8643,  0.0597},
    { 0.6136,  -0.1087,  0.2458,  0.3198,  0.4097,  0.8519,  0.0648},
    { 0.7079,  -0.1330,  0.2277,  0.3485,  0.4112,  0.8400,  0.0613},
    { 0.7597,  -0.1337,  0.1847,  0.4163,  0.3838,  0.8210,  0.0723},
    { 0.5876,  -0.1529,  0.2274,  0.3597,  0.4105,  0.8374,  0.0281},
    { 0.6773,  -0.1682,  0.2051,  0.3731,  0.4055,  0.8341,  0.0256},
    { 0.7569,  -0.1936,  0.2040,  0.4143,  0.3931,  0.8207,  0.0174},
    { 0.5857,  -0.2141,  0.2239,  0.3447,  0.4491,  0.8243,  -0.0107},
    { 0.6853,  -0.2246,  0.1952,  0.3745,  0.4325,  0.8201,  -0.0104},
    { 0.7742,  -0.2100,  0.2008,  0.4236,  0.4554,  0.7821,  0.0378},
    { 0.6301,  -0.1078,  0.3114,  0.2115,  0.4540,  0.8638,  0.0552},
    { 0.7482,  -0.1132,  0.2886,  0.2872,  0.4348,  0.8520,  0.0514},
    { 0.7946,  -0.2048,  0.2890,  0.2475,  0.4898,  0.8360,  -0.0060},
    { 0.6773,  -0.2454,  0.2775,  0.2051,  0.4249,  0.8799,  0.0557},
    { 0.7781,  -0.2667,  0.2831,  0.2328,  0.3981,  0.8855,  0.0559},
};*/
// Travel time to each waypoint (robot max ~0.25 m/s; keep generous).
static const double DT_PER_WAYPOINT_MS = 4000.0;
// How long to hold each pose before moving to the next.
static const double HOLD_MS = 15000.0;
// ─────────────────────────────────────────────────────────────────────────────

void main_print(const std::string& msg) {
  std::cout << "================================================" << std::endl;
  std::cout << "== Main Stage" << std::endl;
  std::cout << "== " << msg << std::endl;
  std::cout << "================================================" << std::endl;
}

int main() {
  const std::string config_path =
      "/home/robotlab/ACP/hardware_interfaces/workcell/"
      "table_top_manip/"
      "config/single_arm_data_collection_franka.yaml";

  ManipServer server(config_path);

  while (!server.is_ready()) {
    std::cout << "[main] Waiting for server to be ready." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
  }

  // Full compliance axes (not used by impedance controller, but set for
  // consistency with the rest of the pipeline).
  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  server.set_force_controlled_axis(Tr, 6, 0);
  if (server.is_bimanual())
    server.set_force_controlled_axis(Tr, 6, 1);

  server.set_high_level_maintain_position();

  // Each pose is scheduled twice: once for arrival, once for hold-end.
  // The interpolator moves linearly between identical points = perfect hold.
  const int N_COLS = N_WAYPOINTS * 2;
  Eigen::MatrixXd waypoints(7, N_COLS);
  for (int i = 0; i < N_WAYPOINTS; ++i) {
    for (int j = 0; j < 7; ++j)
      waypoints(j, 2 * i) = waypoints(j, 2 * i + 1) = WAYPOINT_POSES[i][j];
  }

  std::cout << "[main] Planned waypoints (rows = xyz qwxyz, cols = steps):\n"
            << waypoints << std::endl;

  // Main loop — each iteration is one recorded episode.
  RUT::Timer duration_timer;
  while (true) {
    main_print("Press Enter to start a new scripted-motion episode.");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    duration_timer.tic();

    server.start_saving_data_for_a_new_episode();
    main_print("Recording started. Executing scripted trajectory...");

    // Build absolute timepoints: arrival and hold-end for each pose.
    double t0 = server.get_timestamp_now_ms();
    Eigen::VectorXd timepoints_ms(N_COLS);
    for (int i = 0; i < N_WAYPOINTS; ++i) {
      double t_arrive  = t0 + i * (DT_PER_WAYPOINT_MS + HOLD_MS) + DT_PER_WAYPOINT_MS;
      double t_holdend = t_arrive + HOLD_MS;
      timepoints_ms(2 * i)     = t_arrive;
      timepoints_ms(2 * i + 1) = t_holdend;
    }

    server.schedule_waypoints(waypoints, timepoints_ms, 0);

    // Progress indicator: sleep through travel + hold for each pose.
    for (int i = 0; i < N_WAYPOINTS; ++i) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(DT_PER_WAYPOINT_MS)));
      Eigen::MatrixXd cur = server.get_pose(1, 0);
      std::cout << "[main] Waypoint " << (i + 1) << "/" << N_WAYPOINTS
                << " arrived. Current pose: " << cur.col(0).transpose()
                << std::endl;
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(HOLD_MS)));
      std::cout << "[main] Waypoint " << (i + 1) << "/" << N_WAYPOINTS
                << " hold complete." << std::endl;
    }
    // Extra settling margin — the interpolator holds home via keep_the_last_target
    // with the same high stiffness that was set before recording. Do NOT call
    // set_high_level_maintain_position() here: that call uses getCartesian()
    // (readOnce from main thread) and then clears the cmd buffer + issues new
    // targets, which creates a timeline discontinuity in the interpolation
    // controller that the Franka safety monitor trips on.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    double episode_s = duration_timer.toc_ms() / 1000.0;
    std::cout << "[main] Episode duration: " << episode_s << " s. "
              << "Waiting for threads to flush." << std::endl;

    server.stop_saving_data();
    while (server.is_saving_data())
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "[main] All threads have stopped saving data." << std::endl;

    std::cout << "[main] What to do:\n"
              << "[main]    q  — quit\n"
              << "[main]    any other key — save and run another episode\n";
    char c;
    std::cin.get(c);

    if (c == 'q') {
      std::cout << "[main] Quitting." << std::endl;
      break;
    }
    // Re-establish a clean maintain-position target before the next episode.
    // Robot is already at home; this just refreshes the waypoint buffer.
    std::cout << "[main] Saving and continuing." << std::endl;
    server.set_high_level_maintain_position();
  }

  server.join_threads();
  std::cout << "[main] Threads joined. Exiting." << std::endl;
  return 0;
}