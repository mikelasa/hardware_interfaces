/*
 * Autonomous overnight data collection for wrench bias calibration.
 *
 * Moves the robot to random EE poses, holds each pose for HOLD_TIME_MS with
 * no contact, and saves one ManipServer episode per pose. Runs indefinitely
 * until killed with Ctrl-C (or until N_POSES_MAX is reached).
 *
 * Each episode writes to <data_folder>/<timestamp>/:
 *   joints_data_0.json  — q at 1 kHz             (NN input)
 *   wrench_data_0.json  — O_F_ext_hat_K wrench    (NN target)
 *
 * Post-process with train_wrench_nn.py:
 *   - Extract mean q and mean wrench over the middle 200 samples of each hold
 *   - Train a small MLP:  q(7) → bias(6)
 *   - Export weights for Eigen-based C++ inference
 *
 * Usage: ./wrench_calib_recorder [config_yaml]
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <table_top_manip/manip_server.h>

// ─── Timing [ms] ─────────────────────────────────────────────────────────────
static constexpr double TRAVEL_TIME_MS = 10000;   // budget for the robot to reach the target
static constexpr double SETTLE_TIME_MS = 2500.0;   // extra wait after arrival before recording
static constexpr double HOLD_TIME_MS   = 15000.0;  // static hold (training window)
static constexpr int    N_POSES_MAX    = -1;        // -1 = run overnight indefinitely

// ─── Workspace corners (measured with read_pose) ──────────────────────────────
// Poses are [x, y, z, qw, qx, qy, qz] captured at the 8 reachable extremes.
// Sampling as convex combinations of these corners guarantees every generated
// pose is inside the demonstrated feasible workspace — no joint limit violations.
static constexpr int N_CORNERS = 8;
static constexpr double CORNERS[N_CORNERS][7] = {
    // x       y        z       qw      qx      qy      qz
    { 0.9081, -0.3348,  0.2356, -0.0735, 0.7995, -0.5124,  0.3046 },
    { 0.9324, -0.3117,  0.0076, -0.0983, 0.7914, -0.4957,  0.3439 },
    { 0.6794, -0.3115,  0.0093, -0.0806, 0.7630, -0.5362,  0.3518 },
    { 0.6046, -0.3701,  0.2404, -0.0153, 0.8098, -0.5561,  0.1861 },
    { 0.6425,  0.2723,  0.2622, -0.1066, 0.9614, -0.1908,  0.1670 },
    { 0.6471,  0.2716,  0.0206, -0.2065, 0.9371, -0.1970,  0.2008 },
    { 0.9655,  0.2117,  0.0136, -0.1743, 0.9030, -0.2261,  0.3211 },
    { 0.8804,  0.1947,  0.2407, -0.0932, 0.9280, -0.2958,  0.2066 },
};

// ─── Anti-clustering [m] ─────────────────────────────────────────────────────
static constexpr double MIN_DIST_M = 0.08;  // reject if within this of any recent pose
static constexpr int    CLUSTER_MEM = 200;  // remember last N actual positions

// ─── Dirichlet concentration parameter ───────────────────────────────────────
// Controls where convex-combination samples land in the workspace.
//   alpha = 1.0  → uniform on simplex → most samples cluster near the centroid
//   alpha < 1.0  → sparse weights    → samples spread toward corners and edges
static constexpr double DIRICHLET_ALPHA = 0.4;

static const std::string DEFAULT_CONFIG =
    "/home/robotlab/ACP/hardware_interfaces/workcell/"
    "table_top_manip/config/wrench_calib_franka.yaml";

// ─────────────────────────────────────────────────────────────────────────────

// Generate a random point uniformly on the (N-1)-simplex using the
// exponential-normalisation trick, then return a convex combination of the
// N_CORNERS measured workspace poses.  Every such combination is inside the
// demonstrated feasible workspace, so joint-limit violations cannot occur.
static RUT::Vector7d sample_pose(std::mt19937& rng,
                                  const std::vector<Eigen::Vector3d>& recent) {
    std::uniform_real_distribution<double> ru(0.0, 1.0);
    const Eigen::Quaterniond q_ref(CORNERS[0][3], CORNERS[0][4],
                                   CORNERS[0][5], CORNERS[0][6]);

    for (int attempt = 0; attempt < 300; ++attempt) {
        // ── Dirichlet(alpha,...,alpha) weights ────────────────────────────────
        // alpha < 1 pushes samples toward corners/edges of the workspace,
        // avoiding the centroid clustering of the uniform simplex (alpha=1).
        std::gamma_distribution<double> gamma_dist(DIRICHLET_ALPHA, 1.0);
        double w[N_CORNERS], wsum = 0.0;
        for (int i = 0; i < N_CORNERS; ++i) {
            w[i] = gamma_dist(rng);
            wsum += w[i];
        }
        for (int i = 0; i < N_CORNERS; ++i) w[i] /= wsum;

        // ── Weighted average position ─────────────────────────────────────────
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        for (int i = 0; i < N_CORNERS; ++i)
            pos += w[i] * Eigen::Vector3d(CORNERS[i][0], CORNERS[i][1], CORNERS[i][2]);

        bool too_close = false;
        for (const auto& p : recent)
            if ((pos - p).norm() < MIN_DIST_M) { too_close = true; break; }
        if (too_close) continue;

        // ── Weighted average quaternion (flip to same hemisphere first) ───────
        Eigen::Vector4d qsum = Eigen::Vector4d::Zero();
        for (int i = 0; i < N_CORNERS; ++i) {
            Eigen::Quaterniond qi(CORNERS[i][3], CORNERS[i][4],
                                  CORNERS[i][5], CORNERS[i][6]);
            if (qi.dot(q_ref) < 0.0) qi.coeffs() = -qi.coeffs();
            qsum += w[i] * Eigen::Vector4d(qi.w(), qi.x(), qi.y(), qi.z());
        }
        Eigen::Quaterniond q(qsum(0), qsum(1), qsum(2), qsum(3));
        q.normalize();

        RUT::Vector7d pose;
        pose << pos.x(), pos.y(), pos.z(), q.w(), q.x(), q.y(), q.z();
        return pose;
    }

    // Fallback: centroid of all corners (always feasible)
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector4d qsum = Eigen::Vector4d::Zero();
    for (int i = 0; i < N_CORNERS; ++i) {
        pos += Eigen::Vector3d(CORNERS[i][0], CORNERS[i][1], CORNERS[i][2]);
        Eigen::Quaterniond qi(CORNERS[i][3], CORNERS[i][4],
                              CORNERS[i][5], CORNERS[i][6]);
        if (qi.dot(q_ref) < 0.0) qi.coeffs() = -qi.coeffs();
        qsum += Eigen::Vector4d(qi.w(), qi.x(), qi.y(), qi.z());
    }
    pos /= N_CORNERS;
    Eigen::Quaterniond q(qsum(0), qsum(1), qsum(2), qsum(3));
    q.normalize();
    RUT::Vector7d pose;
    pose << pos.x(), pos.y(), pos.z(), q.w(), q.x(), q.y(), q.z();
    return pose;
}

int main(int argc, char** argv) {
    const std::string config_path = (argc > 1) ? argv[1] : DEFAULT_CONFIG;

    const double secs_per_pose =
        (TRAVEL_TIME_MS + SETTLE_TIME_MS + HOLD_TIME_MS) / 1000.0;

    std::cout << "[calib] Wrench bias calibration recorder\n"
              << "[calib] Config:        " << config_path << "\n"
              << "[calib] Poses target:  "
              << (N_POSES_MAX < 0 ? "∞ (overnight)" : std::to_string(N_POSES_MAX)) << "\n"
              << "[calib] Seconds/pose:  " << secs_per_pose << "\n"
              << "[calib] Est. 1900 poses: "
              << std::fixed << std::setprecision(1)
              << 1900.0 * secs_per_pose / 3600.0 << " h\n\n";

    ManipServer server(config_path);
    while (!server.is_ready()) {
        std::cout << "[calib] Waiting for server...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
    std::cout << "[calib] Server ready. Starting calibration loop.\n\n";

    // All 6 axes position-controlled (no active force control during calibration)
    RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
    server.set_force_controlled_axis(Tr, 6, 0);
    server.set_high_level_maintain_position();

    std::mt19937 rng(std::random_device{}());
    std::vector<Eigen::Vector3d> recent_positions;

    RUT::Timer wall_clock;
    wall_clock.tic();
    int pose_count = 0;

    while (N_POSES_MAX < 0 || pose_count < N_POSES_MAX) {

        // ── 0. Robot health check ─────────────────────────────────────────────
        if (!server.is_running()) {
            std::cout << "[calib] ERROR: ManipServer stopped — robot likely in error state. Exiting.\n";
            break;
        }

        // ── 1. Sample a random candidate pose ────────────────────────────────
        RUT::Vector7d target = sample_pose(rng, recent_positions);

        std::cout << "[calib] ── Pose " << (pose_count + 1) << "\n"
                  << "[calib]    Target xyz: ["
                  << std::fixed << std::setprecision(3)
                  << target(0) << ", " << target(1) << ", " << target(2) << "]\n"
                  << "[calib]    Target quat (w,x,y,z): ["
                  << target(3) << ", " << target(4) << ", "
                  << target(5) << ", " << target(6) << "]\n";

        // ── 2. Schedule: arrive at target, then hold through settle+record ───
        double t0 = server.get_timestamp_now_ms();

        Eigen::MatrixXd waypoints(7, 2);
        waypoints.col(0) = target;
        waypoints.col(1) = target;

        Eigen::VectorXd timepoints_ms(2);
        timepoints_ms(0) = t0 + TRAVEL_TIME_MS;
        timepoints_ms(1) = t0 + TRAVEL_TIME_MS + SETTLE_TIME_MS + HOLD_TIME_MS;

        server.schedule_waypoints(waypoints, timepoints_ms, 0);

        // ── 3. Wait for travel ────────────────────────────────────────────────
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(TRAVEL_TIME_MS)));

        // ── 4. Check where we actually ended up ───────────────────────────────
        Eigen::MatrixXd actual = server.get_pose(1, 0);
        Eigen::Vector3d actual_pos = actual.col(0).head<3>();
        double arrival_error = (actual_pos - target.head<3>()).norm();
        std::cout << "[calib]    Actual  xyz: ["
                  << actual_pos(0) << ", " << actual_pos(1) << ", " << actual_pos(2)
                  << "]  err=" << std::setprecision(4) << arrival_error << " m\n";

        // Large arrival error means the robot didn't move — stopped or errored.
        if (arrival_error > 0.15) {
            std::cout << "[calib] ERROR: arrival error " << arrival_error
                      << " m exceeds 0.15 m threshold — robot likely stopped. Exiting.\n";
            break;
        }

        // ── 5. Extra settle wait ──────────────────────────────────────────────
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(SETTLE_TIME_MS)));

        // ── 6. Record this hold window as one episode ─────────────────────────
        server.start_saving_data_for_a_new_episode();
        std::cout << "[calib]    Recording hold (" << HOLD_TIME_MS / 1000.0 << " s)...\n";

        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(HOLD_TIME_MS)));

        server.stop_saving_data();
        while (server.is_saving_data())
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // ── 7. Update anti-clustering memory ─────────────────────────────────
        recent_positions.push_back(actual_pos);
        if (static_cast<int>(recent_positions.size()) > CLUSTER_MEM)
            recent_positions.erase(recent_positions.begin());

        // ── 8. Progress report ────────────────────────────────────────────────
        pose_count++;
        double elapsed_min = wall_clock.toc_ms() / 60000.0;
        double rate_per_min = static_cast<double>(pose_count) / elapsed_min;
        double eta_min = (1900.0 - pose_count) / rate_per_min;

        std::cout << "[calib]    Saved. Total: " << pose_count
                  << "  Elapsed: " << std::setprecision(1) << elapsed_min << " min"
                  << "  Rate: " << std::setprecision(2) << rate_per_min << " poses/min";
        if (pose_count < 1900)
            std::cout << "  ETA 1900: " << std::setprecision(0) << eta_min << " min";
        std::cout << "\n";

        // Re-establish maintain-position while we compute the next target
        server.set_high_level_maintain_position();
    }

    std::cout << "\n[calib] Done. " << pose_count << " episodes recorded.\n";
    server.join_threads();
    return 0;
}
