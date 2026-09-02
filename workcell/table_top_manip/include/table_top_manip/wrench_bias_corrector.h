/**
 * WrenchBiasCorrector — header-only Eigen MLP for pose-dependent wrench bias.
 *
 * Loads weights exported by
 * PyriteUtility/data_pipeline/wrench_bias_filter/export.py (.bin format)
 * and subtracts the predicted pose-dependent bias from raw wrench measurements.
 *
 * Usage:
 *   WrenchBiasCorrector corrector;
 *   corrector.load("/path/to/model.bin");
 *   RUT::Vector6d bias = corrector.predict(pose_fb.head<3>());
 *   wrench -= bias;
 *
 * Binary layout (all little-endian):
 *   int32   magic      = 0x57424D31 ("WBM1")
 *   int32   input_dim, hidden1, hidden2, output_dim
 *   float64 x_mean[input_dim],  x_std[input_dim]
 *   float64 y_mean[output_dim], y_std[output_dim]
 *   float64 W1[hidden1 * input_dim]   row-major
 *   float64 b1[hidden1]
 *   float64 W2[hidden2 * hidden1]     row-major
 *   float64 b2[hidden2]
 *   float64 W3[output_dim * hidden2]  row-major
 *   float64 b3[output_dim]
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

class WrenchBiasCorrector {
 public:
    WrenchBiasCorrector() = default;

    bool load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) {
            std::cerr << "[WrenchBiasCorrector] cannot open: " << path << "\n";
            return false;
        }

        // ── Magic + architecture ──────────────────────────────────────────────
        int32_t magic, in_dim, h1, h2, out_dim, act_id = 0;
        if (!_read(f, magic)) {
            std::cerr << "[WrenchBiasCorrector] read error in: " << path << "\n";
            return false;
        }
        if (magic == 0x57424D32) {           // WBM2: includes activation field
            _read(f, in_dim); _read(f, h1); _read(f, h2); _read(f, out_dim);
            _read(f, act_id);
        } else if (magic == 0x57424D31) {    // WBM1: legacy, relu assumed
            _read(f, in_dim); _read(f, h1); _read(f, h2); _read(f, out_dim);
        } else {
            std::cerr << "[WrenchBiasCorrector] bad magic in: " << path << "\n";
            return false;
        }
        _act_id = act_id;

        // ── Normalisation stats ───────────────────────────────────────────────
        _x_mean = _read_vec(f, in_dim);
        _x_std  = _read_vec(f, in_dim);
        _y_mean = _read_vec(f, out_dim);
        _y_std  = _read_vec(f, out_dim);

        // ── Weights ───────────────────────────────────────────────────────────
        _W1 = _read_mat(f, h1, in_dim);  _b1 = _read_vec(f, h1);
        _W2 = _read_mat(f, h2, h1);      _b2 = _read_vec(f, h2);
        _W3 = _read_mat(f, out_dim, h2); _b3 = _read_vec(f, out_dim);

        if (f.fail()) {
            std::cerr << "[WrenchBiasCorrector] read error in: " << path << "\n";
            return false;
        }

        _loaded = true;
        _tau_J_filtered.setZero();
        const char* act_name[] = {"relu","gelu","tanh","elu"};
        std::cout << "[WrenchBiasCorrector] loaded " << path << "  arch "
                  << in_dim << "→" << h1 << "→" << h2 << "→" << out_dim
                  << "  act=" << act_name[std::min(act_id,3)] << "\n";
        return true;
    }

    bool is_loaded() const { return _loaded; }

    // Predict the wrench bias for the given joint angles q[7] [rad]
    // and joint torques tau_J[7] [Nm].  Falls back to q-only if the loaded
    // model has input_dim==7 (legacy weights without torques).
    // Returns zero if the model is not loaded.
    Eigen::Matrix<double, 6, 1> predict(const Eigen::Matrix<double, 7, 1>& q,
                                        const Eigen::Matrix<double, 7, 1>& tau_J) const {
        if (!_loaded) return Eigen::Matrix<double, 6, 1>::Zero();

        // Low-pass filter tau_J to match the 10 Hz cutoff on the wrench itself.
        _tau_J_filtered = _TAU_EMA_ALPHA * tau_J + (1.0 - _TAU_EMA_ALPHA) * _tau_J_filtered;

        Eigen::VectorXd x(_x_mean.size());
        if (_x_mean.size() == 14) {
            x << q, _tau_J_filtered;
        } else {
            x = q;  // legacy 7-input model
        }
        Eigen::VectorXd xn = (x - _x_mean).cwiseQuotient(_x_std);
        Eigen::VectorXd h1 = _act(_W1 * xn + _b1);
        Eigen::VectorXd h2 = _act(_W2 * h1 + _b2);
        Eigen::VectorXd yn = _W3 * h2 + _b3;
        return yn.cwiseProduct(_y_std) + _y_mean;
    }

 private:
    bool    _loaded{false};
    int32_t _act_id{0};   // 0=relu, 1=gelu, 2=tanh, 3=elu

    // EMA state for tau_J — matches the 10 Hz lowpass applied to the wrench.
    // α = 1 - exp(-2π·10/1000) ≈ 0.0609  →  10 Hz EMA at 1 kHz
    mutable Eigen::Matrix<double, 7, 1> _tau_J_filtered;
    static constexpr double _TAU_EMA_ALPHA = 0.0609;

    Eigen::VectorXd _x_mean, _x_std, _y_mean, _y_std;
    Eigen::MatrixXd _W1, _W2, _W3;
    Eigen::VectorXd _b1, _b2, _b3;

    Eigen::VectorXd _act(const Eigen::VectorXd& v) const {
        switch (_act_id) {
            case 1: return v.unaryExpr([](double x){  // gelu
                        return x * 0.5 * (1.0 + std::erf(x / 1.41421356237)); });
            case 2: return v.unaryExpr([](double x){ return std::tanh(x); });
            case 3: return v.unaryExpr([](double x){ return x >= 0.0 ? x : std::expm1(x); });
            default: return v.cwiseMax(0.0);  // relu
        }
    }

    template <typename T>
    bool _read(std::ifstream& f, T& val) {
        f.read(reinterpret_cast<char*>(&val), sizeof(T));
        return !f.fail();
    }

    Eigen::VectorXd _read_vec(std::ifstream& f, int n) {
        Eigen::VectorXd v(n);
        f.read(reinterpret_cast<char*>(v.data()), n * sizeof(double));
        return v;
    }

    Eigen::MatrixXd _read_mat(std::ifstream& f, int rows, int cols) {
        // Python writes row-major; Eigen default is column-major, so we read
        // into a temporary row-major matrix then convert.
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tmp(rows, cols);
        f.read(reinterpret_cast<char*>(tmp.data()), rows * cols * sizeof(double));
        return Eigen::MatrixXd(tmp);
    }
};
