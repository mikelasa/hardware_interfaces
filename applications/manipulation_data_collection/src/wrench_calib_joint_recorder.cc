/*
 * Joint-space wrench bias calibration recorder.
 *
 * One continuous robot.control() loop runs for the entire session — no gaps
 * between episodes, no gravity-compensation transitions, no vibration.
 * A separate logic thread handles JSON saving and pose sampling while the
 * robot is already moving to the next target.
 *
 * Architecture:
 * control thread (1 kHz) — state machine: SETTLE → MOVING → HOLDING → SETTLE …
 * logic thread            — save JSON, sample next q, signal control thread
 *
 * Output per episode:
 * joint_data_0.json   — q  at 1 kHz   (NN input)
 * wrench_data_0.json  — O_F_ext_hat_K (NN target)
 *
 * IMPORTANT: ManipServer must NOT be running.
 *
 * Usage: ./wrench_calib_joint_recorder [robot_ip] [data_folder]
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

namespace fs = std::filesystem;

// ─── Timing ──────────────────────────────────────────────────────────────────
static constexpr double HOLD_S        = 15.0;  // hold duration per pose [s]
static constexpr double MAX_JOINT_VEL = 0.3;   // rad/s — conservative overnight
static constexpr double MIN_TRAJ_S    = 2.5;   // minimum move duration [s]
static constexpr double SETTLE_S      = 0.5;   // brief settle after arriving [s]
static constexpr double ARRIVE_THRESH = 0.015; // rad — "arrived" threshold

// ─── Reference joint configurations (captured with read_pose) ────────────────
static constexpr int N_REF = 18;
static constexpr double Q_REF[N_REF][7] = /*{{ -0.2738,  0.6558,  -0.2566,  -2.1829,  -0.1530,  3.5292,  1.0141},
    { -0.1882,  0.4946,  -0.1565,  -2.5234,  -0.1539,  3.7031,  0.9335},
    { -0.0962,  0.4504,  0.0378,  -2.6155,  -0.1539,  3.7033,  0.9812},
    { 0.2335,  0.5695,  0.1109,  -2.4141,  0.2792,  3.7151,  0.6889},
    { 0.2342,  0.7013,  0.1478,  -2.2187,  0.0519,  3.7250,  0.7721},
    { 0.2817,  1.3588,  -0.0365,  -1.1109,  0.0521,  3.5847,  0.7472},
    { 0.1714,  1.5349,  -0.1646,  -0.7980,  0.0818,  3.4552,  0.8152},
    { -0.0103,  1.5471,  -0.1731,  -0.7491,  0.1269,  3.4021,  0.8495},
    { -0.1584,  1.4175,  -0.1470,  -0.9267,  0.1190,  3.3683,  0.8490},
    { -0.2439,  1.2258,  -0.2149,  -1.2576,  0.1349,  3.4157,  0.8895},
    { -0.2413,  0.9310,  -0.1085,  -1.2472,  0.0598,  3.0280,  0.8852},
    { -0.2553,  0.5569,  -0.0935,  -1.8430,  0.1220,  3.0840,  0.8966},
    { -0.2542,  0.2120,  -0.1013,  -2.3893,  0.1189,  3.1968,  0.7982},
    { -0.1942,  0.1064,  0.1333,  -2.4249,  0.1027,  3.1898,  0.7894},
    { -0.1742,  0.2262,  0.5785,  -2.3429,  0.0742,  3.2118,  0.7628},
    { 0.2310,  0.5236,  0.0559,  -1.8708,  0.0293,  3.1034,  0.6859},
    { 0.2295,  0.9860,  -0.0644,  -1.0916,  0.0410,  2.8433,  0.7374},
    { 0.0043,  1.0047,  -0.1318,  -0.9711,  0.1214,  2.6981,  0.7463}
};*/

{
    //   q1       q2       q3       q4       q5       q6       q7
    { 0.1534,  -0.0924,  -0.8464,  -2.2308,  -0.0686,  2.3926,  0.3264},
    { 0.1506,   0.6035,  -0.6821,  -2.1235,   0.8843,  2.7568, -0.1587},
    { 0.1939,   0.3164,  -0.2557,  -2.3991,   0.3232,  2.9255,  0.4337},
    { 0.2103,  -0.4251,  -0.4455,  -2.3810,  -0.2015,  2.2501,  0.7153},
    { 0.2188,  -0.6241,  -0.1158,  -2.5380,  -0.1872,  2.2799,  1.1793},
    { 0.5139,  -0.2667,   0.0404,  -2.2391,  -0.0685,  2.3116,  1.2600},
    { 0.5156,   0.3624,  -0.0793,  -2.3743,  -0.1024,  3.1338,  1.2655},
    { 0.3663,   0.7699,  -0.1067,  -1.9001,   0.0944,  3.4446,  0.8113},
    { 0.3999,   0.2544,  -0.0861,  -1.7510,   0.0729,  2.4335,  0.8575},
    { 0.3146,   1.3856,  -0.2091,  -0.7210,   0.1797,  2.7816,  0.8282},
    { 0.3342,   0.9858,  -0.2381,  -0.4992,   0.1793,  2.0799,  0.8892},
    { 0.0579,   0.5608,  -0.1829,  -1.2553,   0.1389,  2.4326,  0.6240},
    { 0.0517,   1.3115,  -0.1632,  -0.9174,   0.1414,  2.9789,  0.6949},
    {-0.3163,   1.4792,  -0.1342,  -0.5198,   0.2327,  2.6604,  0.6794},
    {-0.2868,   0.8215,  -0.2367,  -0.9203,   0.1553,  2.3249,  0.5988},
    {-0.3413,   0.9101,  -0.1305,  -1.5671,   0.1988,  3.0793,  0.5390},
    {-0.3226,   0.4219,  -0.1825,  -1.5849,   0.1068,  2.5288,  0.6058},
    {-0.2562,  -0.6028,   0.2143,  -2.4650,   0.1198,  2.2762,  0.7261},
};

static constexpr double DIRICHLET_ALPHA = 0.2;

// ─── Anti-clustering ─────────────────────────────────────────────────────────
static constexpr double CLUSTER_DIST = 0.10;
static constexpr int    CLUSTER_MEM  = 50;
static constexpr int max_attempts = 5000;

// ─── Paths ───────────────────────────────────────────────────────────────────
static const std::string DEFAULT_IP     = "172.17.6.164";
static const std::string DEFAULT_FOLDER = "/home/robotlab/data/real/wrench_calibration_test";

static std::atomic<bool> g_shutdown{false};
static void sig_handler(int) { g_shutdown = true; }

// ─────────────────────────────────────────────────────────────────────────────

static std::array<double,7> sample_joints(
    std::mt19937& rng,
    const std::vector<std::array<double,7>>& recent)
{
    std::gamma_distribution<double> gamma(DIRICHLET_ALPHA, 1.0);
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        double w[N_REF], wsum = 0.0;
        for (int i = 0; i < N_REF; ++i) { w[i] = gamma(rng); wsum += w[i]; }
        for (int i = 0; i < N_REF; ++i) w[i] /= wsum;

        std::array<double,7> q{};
        for (int i = 0; i < N_REF; ++i)
            for (int j = 0; j < 7; ++j)
                q[j] += w[i] * Q_REF[i][j];

        bool too_close = false;
        for (const auto& prev : recent) {
            double d = 0.0;
            for (int j = 0; j < 7; ++j) d += (q[j]-prev[j])*(q[j]-prev[j]);
            if (std::sqrt(d) < CLUSTER_DIST) { too_close = true; break; }
        }
        if (!too_close) return q;
    }
    // Fallback: centroid
    std::array<double,7> q{};
    for (int i = 0; i < N_REF; ++i)
        for (int j = 0; j < 7; ++j)
            q[j] += Q_REF[i][j] / N_REF;
    return q;
}

// ─── JSON helpers ─────────────────────────────────────────────────────────────

static void write_arr7(std::ofstream& f, const std::array<double,7>& a) {
    f << "[";
    for (int i = 0; i < 7; ++i) f << std::fixed << std::setprecision(7) << a[i] << (i<6?", ":"");
    f << "]";
}
static void write_arr6(std::ofstream& f, const std::array<double,6>& a) {
    f << "[";
    for (int i = 0; i < 6; ++i) f << std::fixed << std::setprecision(4) << a[i] << (i<5?", ":"");
    f << "]";
}

static void save_episode(
    const fs::path& folder,
    const std::vector<std::array<double,7>>& q_log,
    const std::vector<std::array<double,7>>& tau_log,
    const std::vector<std::array<double,6>>& w_log,
    const std::vector<double>&               t_log)
{
    fs::create_directories(folder);
    {
        std::ofstream f(folder / "joint_data_0.json");
        f << "[\n";
        for (size_t i = 0; i < q_log.size(); ++i) {
            f << "\t{\n\t\t\"seq_id\": " << i
              << ",\n\t\t\"mask\": 0"
              << ",\n\t\t\"robot_time_stamps\": " << std::fixed << std::setprecision(2) << t_log[i]
              << ",\n\t\t\"q\": "; write_arr7(f, q_log[i]); f << ",\n\t}";
            if (i + 1 < q_log.size()) f << ",";
            f << "\n";
        }
        f << "]\n";
    }
    {
        std::ofstream f(folder / "torque_data_0.json");
        f << "[\n";
        for (size_t i = 0; i < tau_log.size(); ++i) {
            f << "\t{\n\t\t\"seq_id\": " << i
              << ",\n\t\t\"mask\": 0"
              << ",\n\t\t\"robot_time_stamps\": " << std::fixed << std::setprecision(2) << t_log[i]
              << ",\n\t\t\"tau_J\": "; write_arr7(f, tau_log[i]); f << ",\n\t}";
            if (i + 1 < tau_log.size()) f << ",";
            f << "\n";
        }
        f << "]\n";
    }
    {
        std::ofstream f(folder / "wrench_data_0.json");
        f << "[\n";
        for (size_t i = 0; i < w_log.size(); ++i) {
            f << "\t{\n\t\t\"seq_id\": " << i
              << ",\n\t\t\"wrench_time_stamps\": " << std::fixed << std::setprecision(2) << t_log[i]
              << ",\n\t\t\"wrench\": "; write_arr6(f, w_log[i]); f << ",\n\t}";
            if (i + 1 < w_log.size()) f << ",";
            f << "\n";
        }
        f << "]\n";
    }
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    const std::string ip     = (argc > 1) ? argv[1] : DEFAULT_IP;
    const std::string folder = (argc > 2) ? argv[2] : DEFAULT_FOLDER;

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    std::cout << "[calib] Joint-space wrench calibration recorder\n"
              << "[calib] Robot: " << ip << "\n"
              << "[calib] Data:  " << folder << "\n\n";

    franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
    robot.setCollisionBehavior(
        {{50,50,50,50,50,50,50}}, {{60,60,60,60,60,60,60}},
        {{50,50,50,50,50,50,50}}, {{60,60,60,60,60,60,60}},
        {{30,30,30,30,30,30}},    {{40,40,40,40,40,40}},
        {{30,30,30,30,30,30}},    {{40,40,40,40,40,40}});

    // ── Shared state between control callback and logic thread ────────────────
    // target_ready: main sets true after providing q_next; callback clears it
    // data_ready:   callback sets true after hold; main clears it after saving
    std::atomic<bool> target_ready{false};
    std::atomic<bool> data_ready{false};
    std::array<double,7> q_next_shared{};
    std::array<double,7> q_actual_shared{};  // actual joints at end of hold

    // Pre-allocated data buffers — callback writes, main reads (no overlap by design)
    std::vector<std::array<double,7>> q_log;   q_log.reserve(16000);
    std::vector<std::array<double,7>> tau_log; tau_log.reserve(16000);
    std::vector<std::array<double,6>> w_log;   w_log.reserve(16000);
    std::vector<double>               t_log;   t_log.reserve(16000);

    // ── Logic thread: sample → signal → save ─────────────────────────────────
    std::mt19937 rng(std::random_device{}());
    std::vector<std::array<double,7>> recent;
    int pose_count = 0;
    auto t_start = std::chrono::steady_clock::now();

    // Provide the very first target before starting the control loop
    q_next_shared = sample_joints(rng, recent);
    target_ready.store(true);

    std::thread logic_thread([&]() {
        while (!g_shutdown) {
            // Wait for hold to complete
            while (!data_ready.load() && !g_shutdown.load())
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (g_shutdown) break;

            // Copy data (callback is in SETTLE, not touching logs)
            auto q_copy   = q_log;
            auto tau_copy = tau_log;
            auto w_copy   = w_log;
            auto t_copy   = t_log;
            std::array<double,7> q_actual = q_actual_shared;
            data_ready.store(false);

            // Save to JSON
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            fs::path ep_dir = fs::path(folder) / ("episode_" + std::to_string(now_ms));
            save_episode(ep_dir, q_copy, tau_copy, w_copy, t_copy);

            // Anti-clustering + progress
            recent.push_back(q_actual);
            if (static_cast<int>(recent.size()) > CLUSTER_MEM)
                recent.erase(recent.begin());

            pose_count++;
            double elapsed_min = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count() / 60.0;
            std::cout << "[calib] Saved ep " << pose_count
                      << "  " << q_copy.size() << " frames"
                      << "  Elapsed: " << std::fixed << std::setprecision(1) << elapsed_min << " min"
                      << "  Rate: " << std::setprecision(2) << pose_count / elapsed_min << " poses/min\n";

            // Sample and provide next target
            std::array<double,7> q_next = sample_joints(rng, recent);
            std::cout << "[calib] Next q = [";
            for (int i = 0; i < 7; ++i)
                std::cout << std::fixed << std::setprecision(3) << q_next[i] << (i<6?", ":"");
            std::cout << "]\n";

            q_next_shared = q_next;
            target_ready.store(true);  // signal callback to start moving
        }
    });

    // ── Single continuous control loop ────────────────────────────────────────
    enum Phase { SETTLE, MOVING, HOLDING };
    Phase phase = SETTLE;

    std::array<double,7> q_start{}, q_target{};
    double phase_t = 0.0;  // time since phase start
    double traj_dur = 0.0;
    bool   first_tick = true;

    try {
        robot.control([&](const franka::RobotState& state, franka::Duration dt)
                      -> franka::JointPositions {

            double dt_s = dt.toSec();
            phase_t += dt_s;

            if (first_tick) {
                q_start = state.q;
                first_tick = false;
            }

            std::array<double,7> q_cmd = q_start;  // default: hold current

            switch (phase) {

            case SETTLE:
                q_cmd = q_start;
                if (phase_t >= SETTLE_S && target_ready.load()) {
                    // Read next target and begin trajectory
                    
                    // q_start  = state.q; // <--- REMOVED: Keep continuity with previous commanded target
                    
                    q_target = q_next_shared;
                    target_ready.store(false);

                    double max_delta = 0.0;
                    for (int i = 0; i < 7; ++i)
                        max_delta = std::max(max_delta, std::abs(q_target[i] - q_start[i]));
                    traj_dur = std::max(max_delta / MAX_JOINT_VEL + 0.5, MIN_TRAJ_S);

                    phase_t = 0.0;
                    phase   = MOVING;

                    std::cout << "[calib] Moving  q = [";
                    for (int i = 0; i < 7; ++i)
                        std::cout << std::fixed << std::setprecision(3)
                                  << q_target[i] << (i<6?", ":"");
                    std::cout << "]  dur=" << traj_dur << "s\n";
                }
                break;

            case MOVING: {
                // Quintic smoothstep: zero velocity AND acceleration at endpoints
                double t      = std::min(phase_t / traj_dur, 1.0);
                double smooth = t * t * t * (t * (6.0 * t - 15.0) + 10.0);
                for (int i = 0; i < 7; ++i)
                    q_cmd[i] = q_start[i] + smooth * (q_target[i] - q_start[i]);

                if (t >= 1.0) {
                    bool arrived = true;
                    for (int i = 0; i < 7; ++i)
                        if (std::abs(state.q[i] - q_target[i]) > ARRIVE_THRESH)
                            { arrived = false; break; }
                    if (arrived) {
                        std::cout << "[calib] Arrived. Holding...\n";
                        q_log.clear();
                        tau_log.clear();
                        w_log.clear();
                        t_log.clear();
                        phase_t = 0.0;
                        phase   = HOLDING;
                    }
                }
                break;
            }

            case HOLDING:
                q_cmd = q_target;
                q_log.push_back(state.q);
                tau_log.push_back(state.tau_J);
                w_log.push_back(state.O_F_ext_hat_K);
                t_log.push_back(phase_t * 1000.0);

                if (phase_t >= HOLD_S) {
                    q_actual_shared = state.q;
                    data_ready.store(true);   // wake logic thread
                    q_start  = q_target;      // start next SETTLE from here
                    phase_t  = 0.0;
                    phase    = SETTLE;
                }
                break;
            }

            if (g_shutdown)
                return franka::MotionFinished(franka::JointPositions(q_cmd));

            return franka::JointPositions(q_cmd);
        });

    } catch (const franka::Exception& e) {
        std::cerr << "\n[calib] ROBOT ERROR: " << e.what() << "\n";
        g_shutdown = true;
    }

    g_shutdown = true;
    logic_thread.join();
    std::cout << "\n[calib] Stopped. " << pose_count << " episodes recorded.\n";
    return 0;
}