# RPLidar C1 Connection Testing

This tutorial covers testing the connection and functionality of a C1 RPLidar sensor with Perseus. The RPLidar provides 2D laser scan data essential for navigation and mapping.

## Prerequisites

The following conditions must be met before beginning:

- The laptop runs a Linux distribution with the latest code from the perseus-v2 Github repository:
  ```console
  git checkout main
  git pull
  ```
- RPLidar C1 is physically connected via USB
- Required fonts are installed for rendering the terminal with the [`powerlevel10k`](https://github.com/romkatv/powerlevel10k) theme (available in /perseus-v2/fonts)

## Summary

1. Verify hardware connection
2. Check device permissions
3. Launch RPLidar driver
4. Verify scan data
5. Test with visualization

## Hardware Connection Verification

Before launching the RPLidar software, verify the hardware connection:

### Check USB Devices

```console
lsusb
```

Look for the RPLidar device in the output. You should see a USB-to-serial adapter.

### Check Serial Devices

```console
ls -la /dev/ttyUSB*
ls -la /dev/ttyACM*
```

The RPLidar typically appears as `/dev/ttyUSB0` or similar.

### Check Device Permissions

```console
ls -la /dev/ttyUSB0
```

If you don't have read/write permissions, you can temporarily fix them:

```console
sudo chmod 666 /dev/ttyUSB0
```

For a permanent solution, add your user to the `dialout` group:

```console
sudo usermod -a -G dialout $USER
```

Then log out and log back in for the changes to take effect. To apply the group change immediately without logging out, use:

```console
newgrp dialout
```

Alternatively, create a udev rule for persistent permissions:

```console
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

:::{warning}
`MODE="0666"` grants read/write permissions to all users on the system (owner, group, and others). This is convenient but may be a security concern in multi-user environments. For better security, use `MODE="0660", GROUP="dialout"` instead and ensure your user is in the dialout group.
:::

## RPLidar Driver Testing

### Basic Driver Launch

Launch the RPLidar driver with default settings:

```console
nix develop --command ros2 launch rplidar_ros rplidar_c1_launch.py
```

This starts the RPLidar node with these default parameters:

- Serial port: `/dev/ttyUSB0`
- Baudrate: `460800`
- Frame ID: `laser`
- Scan mode: `Standard`

### Custom Parameters

If your RPLidar uses a different port or requires custom settings:

```console
nix develop --command ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyUSB1 serial_baudrate:=460800
```

## Data Verification

After launching the driver, verify that scan data is being published:

### Check Active Nodes

```console
ros2 node list
```

You should see `/rplidar_node` in the output.

### Monitor Scan Data

```console
ros2 topic echo /scan
```

This displays real-time laser scan data. Press `Ctrl+C` to stop.

### Check Topic Information

```console
ros2 topic info /scan
ros2 topic hz /scan
```

The scan frequency should be around 10-15 Hz for normal operation.

## Visualization Testing

For visual confirmation of scan data, launch with RViz:

```console
nix develop --command ros2 launch rplidar_ros view_rplidar_c1_launch.py
```

This launches both the RPLidar driver and RViz. In RViz:

1. You should see a circular pattern of laser points around the origin
2. The laser data updates in real-time as objects move around the sensor
3. The coordinate frame should be labeled as `laser`

## Motor Control

The RPLidar driver provides services to control the sensor's motor:

### Stop Motor

```console
ros2 service call /stop_motor std_srvs/srv/Empty
```

### Start Motor

```console
ros2 service call /start_motor std_srvs/srv/Empty
```

## Troubleshooting

### No Scan Data

1. Verify USB connection and device permissions
2. Check that the motor is spinning (you should hear it)
3. Ensure no other processes are using the serial port
4. Try unplugging and reconnecting the USB cable

### Permission Denied Errors

```console
sudo usermod -a -G dialout $USER
```

Then log out and log back in for the changes to take effect.

### Wrong Serial Port

If `/dev/ttyUSB0` doesn't exist, check for other devices:

```console
dmesg | grep tty
```

This shows recent serial device connections.

## Integration with Perseus

The RPLidar data integrates with Perseus navigation through:

- Topic remapping: `/scan` data is used by navigation algorithms
- Frame transforms: The `laser` frame is linked to the robot's base frame
- Sensor fusion: Combined with other sensors for autonomous navigation
