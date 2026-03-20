# AS7343 Spectral Sensor Driver

ROS 2 driver for the **ams-OSRAM AS7343** 14-channel multi-spectral sensor over I2C, designed for Raspberry Pi 4 and Raspberry Pi 5.

## Overview

The AS7343 measures light intensity across 14 spectral channels spanning 405 nm (violet) to 855 nm (near-infrared), plus broadband visible and flicker detection channels. This package provides a ROS 2 node that exposes the sensor as on-demand services — readings are taken only when a client calls the service.

## Spectral Channels

| Channel | Wavelength | FWHM         | Region              |
| ------- | ---------- | ------------ | ------------------- |
| F1      | 405 nm     | ~30 nm       | Violet              |
| F2      | 425 nm     | ~22 nm       | Violet-Blue         |
| FZ      | 450 nm     | ~55 nm       | Blue (wide)         |
| F3      | 475 nm     | ~30 nm       | Blue-Cyan           |
| F4      | 515 nm     | ~40 nm       | Green-Cyan          |
| F5      | 550 nm     | ~35 nm       | Green               |
| FY      | 555 nm     | ~100 nm      | Yellow-Green (wide) |
| FXL     | 600 nm     | ~80 nm       | Orange (wide)       |
| F6      | 640 nm     | ~50 nm       | Red                 |
| F7      | 690 nm     | ~55 nm       | Deep Red            |
| F8      | 745 nm     | ~50 nm       | Near-IR             |
| NIR     | 855 nm     | ~20 nm       | Near-IR             |
| VIS     | Broadband  | Full visible | Clear channel       |
| FD      | N/A        | N/A          | Flicker detection   |

## Hardware

- **Sensor:** AS7343 (I2C address `0x39`)
- **Breakout boards:** SparkFun AS7343 Qwiic, Adafruit AS7343, Pimoroni AS7343
- **Host:** Raspberry Pi 4 / Raspberry Pi 5
- **Wiring:** VCC→3.3V, GND→GND, SDA→GPIO2, SCL→GPIO3

Verify sensor detection:

```bash
sudo apt-get install -y i2c-tools
i2cdetect -y 1
# Should show 0x39
```

## Build

```bash
cd ~/perseus-v2/software/ros_ws
colcon build --packages-select perseus_interfaces as7343_driver
source install/setup.bash
```

## Run

```bash
# Using launch file (loads default config)
ros2 launch as7343_driver as7343.launch.py

# With overrides
ros2 launch as7343_driver as7343.launch.py i2c_bus:=/dev/i2c-1 required:=true

# Direct run
ros2 run as7343_driver as7343_node --ros-args -p gain:=128
```

## Services

| Service                | Type                                       | Description                                |
| ---------------------- | ------------------------------------------ | ------------------------------------------ |
| `~/get_spectral_data`  | `perseus_interfaces/srv/GetSpectralData`   | Read 14-channel spectral data from sensor  |
| `~/get_flicker_status` | `perseus_interfaces/srv/GetFlickerStatus`  | Read ambient light flicker detection status |

### Calling the Spectral Data Service

**CLI:**

```bash
ros2 service call /as7343_node/get_spectral_data perseus_interfaces/srv/GetSpectralData
```

**Response format:**

```yaml
f1_405nm: 1234
f2_425nm: 2345
fz_450nm: 3456
f3_475nm: 4567
f4_515nm: 5678
f5_550nm: 6789
fy_555nm: 7890
fxl_600nm: 8901
f6_640nm: 9012
f7_690nm: 1023
f8_745nm: 2034
nir_855nm: 3045
vis_clear: 4056
fd_flicker: 5067
analog_saturation: false
digital_saturation: false
data_valid: true
integration_time_ms: 50.04
gain: 256
success: true
message: 'OK'
```

**C++ client example:**

```cpp
#include <perseus_interfaces/srv/get_spectral_data.hpp>

auto client = node->create_client<perseus_interfaces::srv::GetSpectralData>(
    "/as7343_node/get_spectral_data");

auto request = std::make_shared<perseus_interfaces::srv::GetSpectralData::Request>();
auto future = client->async_send_request(request);

if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
{
    auto response = future.get();
    if (response->success)
    {
        RCLCPP_INFO(node->get_logger(), "F1 (405nm): %u", response->f1_405nm);
        RCLCPP_INFO(node->get_logger(), "NIR (855nm): %u", response->nir_855nm);
        RCLCPP_INFO(node->get_logger(), "Integration time: %.2f ms", response->integration_time_ms);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service failed: %s", response->message.c_str());
    }
}
```

**Python client example:**

```python
import rclpy
from rclpy.node import Node
from perseus_interfaces.srv import GetSpectralData

rclpy.init()
node = Node('spectral_client')
client = node.create_client(GetSpectralData, '/as7343_node/get_spectral_data')

client.wait_for_service()
future = client.call_async(GetSpectralData.Request())
rclpy.spin_until_future_complete(node, future)

response = future.result()
if response.success:
    print(f"F1 (405nm): {response.f1_405nm}")
    print(f"NIR (855nm): {response.nir_855nm}")
    print(f"Gain: {response.gain}x")
    print(f"Integration time: {response.integration_time_ms:.2f} ms")
    print(f"Data valid: {response.data_valid}")
else:
    print(f"Error: {response.message}")

node.destroy_node()
rclpy.shutdown()
```

### Calling the Flicker Status Service

**CLI:**

```bash
ros2 service call /as7343_node/get_flicker_status perseus_interfaces/srv/GetFlickerStatus
```

**Response format:**

```yaml
detected_frequency_hz: 100
hz_100_valid: true
hz_120_valid: true
hz_100_detected: true
hz_120_detected: false
fd_saturation: false
fd_valid: true
success: true
message: 'OK'
```

### Inspecting Service Definitions

```bash
# View the full service definition
ros2 interface show perseus_interfaces/srv/GetSpectralData
ros2 interface show perseus_interfaces/srv/GetFlickerStatus

# List available services when the node is running
ros2 service list | grep as7343
```

## Key Parameters

| Parameter                   | Default      | Description                         |
| --------------------------- | ------------ | ----------------------------------- |
| `i2c_bus`                   | `/dev/i2c-1` | I2C bus path                        |
| `device_address`            | `0x39`       | I2C address                         |
| `gain`                      | `256`        | ADC gain (1–2048)                   |
| `atime`                     | `29`         | Integration time multiplier (0–255) |
| `astep`                     | `599`        | Integration step (0–65535)          |
| `smux_mode`                 | `18`         | Channels: 6, 12, or 18              |
| `led_enabled`               | `false`      | Enable on-board LED                 |
| `flicker_detection_enabled` | `true`       | Enable flicker detection            |

Integration time: `(atime + 1) × (astep + 1) × 2.78 µs` — default ~50 ms per cycle, ~150 ms for 18-channel mode.

## Documentation

Full Sphinx documentation is in `docs/`. Build with:

```bash
cd docs && sphinx-build -b html . _build/html
```
