Usage
=====

Building
--------

Build the package with colcon::

    cd ~/perseus-v2/software/ros_ws
    colcon build --packages-select perseus_interfaces as7343_driver
    source install/setup.bash

Running
-------

Using the launch file (recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The launch file loads the default YAML configuration::

    ros2 launch as7343_driver as7343.launch.py

Override parameters at launch::

    ros2 launch as7343_driver as7343.launch.py i2c_bus:=/dev/i2c-3 required:=true

Using a custom config file::

    ros2 launch as7343_driver as7343.launch.py config_file:=/path/to/my_config.yaml

Running the node directly
^^^^^^^^^^^^^^^^^^^^^^^^^

::

    ros2 run as7343_driver as7343_node --ros-args \
        -p i2c_bus:=/dev/i2c-1 \
        -p device_address:=57 \
        -p gain:=256 \
        -p publish_rate_hz:=5.0

Note: when passing ``device_address`` on the command line, use the decimal value (57 = 0x39).

Running with a YAML config
^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    ros2 run as7343_driver as7343_node --ros-args \
        --params-file /path/to/as7343_config.yaml

Verifying Operation
-------------------

Check that the node is running::

    ros2 node list
    # Should show /as7343_node

List the topics::

    ros2 topic list
    # /as7343_node/spectral_data
    # /as7343_node/flicker_status
    # /as7343_node/integration_time_ms

Echo spectral data::

    ros2 topic echo /as7343_node/spectral_data

Check publish rate::

    ros2 topic hz /as7343_node/spectral_data

Troubleshooting
---------------

**Node fails to start with "Failed to initialize"**

- Verify sensor is detected: ``i2cdetect -y 1``
- Check I2C permissions (see Hardware Setup)
- If ``required`` is ``false``, the node will start but won't publish data

**All channel values are 0**

- Check wiring — ensure SDA/SCL are connected correctly
- Try reducing gain if all values are saturated (all 65535)

**Saturation warnings**

- Reduce ``gain`` parameter (e.g., from 256 to 64)
- Reduce ``atime`` or ``astep`` to shorten integration time
- Move sensor further from bright light source

**Low publish rate**

- In 18-channel mode, each reading takes ~3x the integration time
- Default: ~150 ms per reading, max ~6.6 Hz
- To increase rate: reduce ``atime``/``astep`` or switch to 6-channel mode
