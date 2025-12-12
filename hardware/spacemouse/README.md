# SpaceMouse Hardware Interface

This module provides a C++ wrapper around the SpaceMouse 3D input device using the HID API library.

## Overview

The SpaceMouse provides 6-degree-of-freedom (6DoF) input consisting of:
- **3 Translation axes**: X, Y, Z (typically in mm or normalized units)
- **3 Rotation axes**: Roll, Pitch, Yaw (typically in degrees or normalized units)
- **2 Buttons**: Top button and menu button

## Dependencies

- `libhidapi-dev`: HID API library for device communication
  ```bash
  sudo apt-get install libhidapi-dev
  ```

## Features

- **Non-blocking HID input**: Reads device in separate thread
- **Dead zone filtering**: Eliminates noise in low-amplitude inputs
- **Low-pass filtering**: Smooths rapid input fluctuations
- **Configurable scaling**: Adjust sensitivity of translation and rotation axes
- **Device enumeration**: Supports multiple SpaceMouse devices

## Configuration

Add SpaceMouse configuration to your YAML config file:

```yaml
spacemouse0:
  # HID device parameters (defaults for SpaceMouse Pro)
  vendor_id: 0x046d          # Logitech vendor ID
  product_id: 0xc603         # SpaceMouse Pro product ID
  device_index: 0            # Device index if multiple connected
  
  # Input scaling
  translation_scale: 1.0     # Scale factor for translation axes
  rotation_scale: 1.0        # Scale factor for rotation axes
  
  # Filtering
  dead_zone: 0.05            # Dead zone threshold (0-1)
  enable_filtering: true     # Enable low-pass filtering
  filter_alpha: 0.3          # Low-pass filter coefficient (0-1)
  
  # Device update rate
  update_rate_hz: 50         # Read frequency from device
```

## Finding Your Device IDs

To find your SpaceMouse vendor and product IDs:

```bash
lsusb | grep -i "3Dconnexion\|Logitech"
# or
lsusb -v | grep -A2 "3DConnexion\|SpaceMouse"
```

Common SpaceMouse models:
- **SpaceMouse Pro**: `046d:c603`
- **SpaceMouse Wireless**: `046d:c62a`
- **SpaceMouse Compact**: `046d:c603`
- **SpaceNavigator**: `046d:c626`

## Usage in Code

```cpp
#include "spacemouse/spacemouse.h"

// Create and initialize
SpaceMouse spacemouse;
SpaceMouse::SpaceMouseConfig config;
config.vendor_id = 0x046d;
config.product_id = 0xc603;

if (!spacemouse.init(config)) {
  std::cerr << "Failed to initialize SpaceMouse\n";
  return false;
}

// Read 6D input
Eigen::Vector3d translation, rotation;
if (spacemouse.get_6d_input(translation, rotation)) {
  std::cout << "Translation: " << translation.transpose() << "\n";
  std::cout << "Rotation: " << rotation.transpose() << "\n";
}

// Get detailed data including buttons
SpaceMouseData data;
if (spacemouse.get_data(data)) {
  std::cout << "TX: " << data.tx << ", TY: " << data.ty << ", TZ: " << data.tz << "\n";
  std::cout << "Button 1: " << data.buttons[0] << "\n";
  std::cout << "Button 2: " << data.buttons[1] << "\n";
}
```

## HID Report Format

The SpaceMouse communicates via HID reports:

**Motion Report (ID 0x01):**
- Bytes 1-2: X translation (little-endian signed 16-bit)
- Bytes 3-4: Y translation
- Bytes 5-6: Z translation
- Bytes 7-8: X rotation
- Bytes 9-10: Y rotation
- Bytes 11-12: Z rotation

**Button Report (ID 0x03):**
- Byte 13: Button 1 state
- Byte 14: Button 2 state

Note: The exact format may vary depending on the SpaceMouse model. Adjust the parsing in `parse_hid_report()` if needed.

## Troubleshooting

**Device not found:**
- Check device is connected: `lsusb`
- Verify vendor/product IDs match your device
- Check HID API permissions: `ls -la /dev/hidraw*`

**Permissions error:**
- Add user to input group: `sudo usermod -aG input $USER`
- Or use udev rules for automatic permissions

**No data received:**
- Check the HID report format matches your device
- Enable debug output in `parse_hid_report()`
- Verify dead zone and filter parameters

## Author

Your Name <your.email@domain.com>
