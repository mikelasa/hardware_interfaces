# GELLO Hardware Interface

Pure C++ teleoperation interface for the [GELLO](https://github.com/wuphilipp/gello_software) device. Reads joint angles directly from the Dynamixel servos via the DynamixelSDK C++ library — no Python, no ROS, no ZMQ required.

---

## Architecture

GELLO is a passive joint-space device: it has one Dynamixel servo per robot joint plus one for the gripper. The interface opens a serial port to the U2D2 USB adapter, disables servo torque (so the arm moves freely), and streams joint positions from all servos at ~500 Hz in a background thread.

```
GELLO device  ──USB──  U2D2 adapter  ──/dev/ttyACM0──  GelloInterface (C++)
                                                              │
                                                     background read thread
                                                              │
                                                    GroupSyncRead (Protocol 2.0)
                                                              │
                                                     GelloData (joint_positions, gripper)
```

This interface does **not** inherit from `TeleopInterface` because GELLO outputs joint angles (N-DOF), not 6D Cartesian data. It is a standalone class you link into your application alongside the other hardware libraries.

---

## Dependencies

| Dependency | Version | Install |
|---|---|---|
| DynamixelSDK C++ | 2.0 | build from source (see below) |
| yaml-cpp | any | already in hardware_interfaces |
| Eigen3 | ≥3.4 | already in hardware_interfaces |

---

## Installation

### 1. Get the DynamixelSDK source

The SDK is bundled as a submodule inside the gello_software repo:

```bash
cd ~/ACP/gello_software
git submodule init
git submodule update -- third_party/DynamixelSDK
```

### 2. Build and install the C++ library

```bash
cd ~/ACP/gello_software/third_party/DynamixelSDK/c++/build/linux64
make
make install INSTALL_ROOT=$HOME/.local
```

> The `ldconfig` error at the end is harmless — the `.so` and headers are copied correctly to `~/.local/lib` and `~/.local/include/dynamixel_sdk/`.

### 3. USB port permissions

The U2D2 adapter appears as `/dev/ttyUSB*`. Add your user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
newgrp dialout   # or log out and back in
```

### 4. Build hardware_interfaces

The `gello` subdirectory is already registered in the root `CMakeLists.txt`. A normal build picks it up:

```bash
cd ~/ACP/hardware_interfaces/build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local \
         -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
         -DCMAKE_BUILD_TYPE=Debug \
         -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j
sudo make install
```

CMake looks for `libdxl_x64_cpp` in `~/.local/lib` first, then `/usr/local/lib`.

---

## API

### Data structure

```cpp
struct GelloData {
  std::vector<double> joint_positions;  // radians, offsets and signs applied
  double gripper;                       // 0.0 = open, 1.0 = closed
  uint64_t timestamp;                   // microseconds (steady_clock)
};
```

### Configuration

```cpp
struct GelloConfig {
  std::string port{"/dev/ttyACM0"};
  int baudrate{57600};

  std::vector<int>    joint_ids;        // Dynamixel IDs, base → wrist (e.g. {1,2,3,4,5,6,7})
  std::vector<double> joint_offsets;    // per-joint offset in radians (multiples of π/2)
  std::vector<int>    joint_signs;      // +1 or -1 per joint

  int    gripper_id{8};
  double gripper_open_rad;              // servo position (rad) when gripper is open
  double gripper_close_rad;            // servo position (rad) when gripper is closed

  double smoothing_alpha{0.99};        // exponential smoothing (0 = none, 1 = full)
  int    read_rate_hz{500};
};
```

> **Where to find joint_offsets and joint_signs for your robot:**
> `~/ACP/gello_software/gello/agents/gello_agent.py` → `PORT_CONFIG_MAP`
> Each entry is keyed by USB serial port path and contains the calibrated values for that robot.

### Minimal usage example

```cpp
#include <gello/gello.h>

GelloInterface gello;

GelloInterface::GelloConfig cfg;
cfg.port             = "/dev/ttyACM0";
cfg.baudrate         = 57600;
cfg.joint_ids        = {1, 2, 3, 4, 5, 6, 7};
cfg.joint_offsets    = {3*M_PI/2, M_PI, M_PI/2, 2*M_PI, M_PI/2, 3*M_PI/2, 2*M_PI};
cfg.joint_signs      = {1, -1, 1, 1, 1, -1, 1};
cfg.gripper_id       = 8;
cfg.gripper_open_rad  = 195.0 * M_PI / 180.0;
cfg.gripper_close_rad = 152.0 * M_PI / 180.0;

gello.init(cfg);

GelloData data;
while (running) {
    gello.get_data(data);
    // data.joint_positions[0..6]  — arm joints in radians
    // data.gripper                — 0.0 open, 1.0 closed
}

gello.cleanup();
```

### CMake integration

```cmake
find_library(GELLO GELLO HINTS ${CMAKE_INSTALL_PREFIX}/lib)

target_link_libraries(your_target
  GELLO
)
target_include_directories(your_target PRIVATE
  ${CMAKE_INSTALL_PREFIX}/include
)
```

---

## Running the test binary

```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH

# With GELLO plugged in:
./build/hardware/gello/test_gello
```

Prints joint positions and gripper value at 20 Hz for 10 seconds. The default offsets/signs in the test are set for a 7-DOF xArm-style GELLO — edit `src/gello_test.cpp` to match your calibration before running.

---

## Files

```
hardware/gello/
├── CMakeLists.txt
├── README.md                   ← this file
├── include/gello/
│   └── gello.h                 ← GelloData + GelloInterface + GelloConfig
└── src/
    ├── gello.cpp               ← DynamixelSDK read loop, smoothing, offset/sign correction
    └── gello_test.cpp          ← standalone test binary
```