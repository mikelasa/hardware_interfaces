#ifndef _GAMEPAD_HEADER_
#define _GAMEPAD_HEADER_

#include <hardware_interfaces/teleop_interface.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>

/**
 * GamepadData: Structure to hold Xbox controller input
 */
struct GamepadData {
  // Analog sticks (left and right)
  // Range: -1.0 to 1.0 for each axis
  double left_stick_x;   // Left stick horizontal
  double left_stick_y;   // Left stick vertical
  double right_stick_x;  // Right stick horizontal
  double right_stick_y;  // Right stick vertical

  // Triggers (LT and RT)
  // Range: 0.0 to 1.0
  double left_trigger;
  double right_trigger;

  // Digital buttons (0 or 1)
  int button_a;      // Green
  int button_b;      // Red
  int button_x;      // Blue
  int button_y;      // Yellow
  int button_lb;     // Left bumper
  int button_rb;     // Right bumper
  int button_back;   // Back button
  int button_start;  // Start button
  int button_left_stick;   // Left stick pressed
  int button_right_stick;  // Right stick pressed

  // D-pad
  int dpad_up;
  int dpad_down;
  int dpad_left;
  int dpad_right;

  uint64_t timestamp;

  GamepadData()
      : left_stick_x(0), left_stick_y(0), right_stick_x(0), right_stick_y(0),
        left_trigger(0), right_trigger(0), button_a(0), button_b(0),
        button_x(0), button_y(0), button_lb(0), button_rb(0),
        button_back(0), button_start(0), button_left_stick(0),
        button_right_stick(0), dpad_up(0), dpad_down(0), dpad_left(0),
        dpad_right(0), timestamp(0) {}
};

/**
 * Gamepad: Xbox controller interface using Linux Joystick API
 */
class Gamepad : public TeleopInterface {
 public:
  struct GamepadConfig {
    std::string device_path{"/dev/input/js0"};  // Default to first joystick
    double deadzone_stick{0.2};                 // Deadzone for analog sticks (0-1)
    double deadzone_trigger{0.05};              // Deadzone for triggers (0-1)
    int update_rate_hz{100};                    // Update frequency

    bool deserialize(const YAML::Node& node) {
      try {
        if (node["device_path"]) device_path = node["device_path"].as<std::string>();
        if (node["deadzone_stick"]) deadzone_stick = node["deadzone_stick"].as<double>();
        if (node["deadzone_trigger"]) deadzone_trigger = node["deadzone_trigger"].as<double>();
        if (node["update_rate_hz"]) update_rate_hz = node["update_rate_hz"].as<int>();
        return true;
      } catch (const std::exception& e) {
        std::cerr << "Gamepad config error: " << e.what() << std::endl;
        return false;
      }
    }
  };

  Gamepad();
  ~Gamepad();

  bool init(const GamepadConfig& config);

  // function from TeleopInterface
  bool init() override { return false; }  // Use init(config) instead
  bool cleanup() override;
  bool get_data(TeleopData& data) override;
  bool is_connected() const override;

  // Gamepad-specific methods
  bool get_data(GamepadData& data);
  std::string get_device_info() const;

 private:
  void read_loop();
  void parse_joystick_event(const js_event& event);
  void apply_deadzone(double& value, double threshold);

  int device_fd_{-1};                    // File descriptor for /dev/input/jsX
  GamepadData current_data_;
  GamepadConfig config_;

  std::thread read_thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex data_mutex_;
  std::atomic<bool> connected_{false};
  std::string device_info_;
};

#endif  // _GAMEPAD_HEADER_
