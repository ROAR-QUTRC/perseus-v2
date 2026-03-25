Configuration
=============

All parameters are declared as ROS 2 parameters and can be set via YAML config file, launch arguments, or command-line ``--ros-args``.

Default configuration file: ``config/as7343_config.yaml``

I2C Parameters
--------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 15 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``i2c_bus``
     - string
     - ``/dev/i2c-1``
     - Path to the I2C bus device file. On Raspberry Pi 4/5, this is typically ``/dev/i2c-1``.
   * - ``device_address``
     - int
     - ``0x39`` (57)
     - 7-bit I2C address of the AS7343. The default is ``0x39``. Range: 0–127.
   * - ``required``
     - bool
     - ``false``
     - If ``true``, the node will terminate on initialization failure. If ``false``, it continues running but publishes nothing.
   * - ``retry_count``
     - int
     - ``3``
     - Number of initialization retry attempts before giving up.

Timing Parameters
-----------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 15 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``publish_rate_hz``
     - double
     - ``5.0``
     - Rate at which spectral data is published. Limited by integration time (see below).
   * - ``frame_id``
     - string
     - ``as7343_link``
     - TF frame ID for the sensor, used in message headers.

Sensor Parameters
-----------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 15 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``gain``
     - int
     - ``256``
     - ADC gain multiplier. Valid values: 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048. Higher gain increases sensitivity but risks saturation.
   * - ``atime``
     - int
     - ``29``
     - Integration time multiplier (0–255). See integration time formula below.
   * - ``astep``
     - int
     - ``599``
     - Integration step period (0–65535). See integration time formula below.
   * - ``smux_mode``
     - int
     - ``18``
     - Number of spectral channels to read. ``6`` = single cycle (fast), ``12`` = two cycles, ``18`` = all channels (recommended).
   * - ``led_enabled``
     - bool
     - ``false``
     - Enable the on-board LED for reflectance measurements.
   * - ``led_current_ma``
     - int
     - ``4``
     - LED drive current in milliamps. Range: 4–258 (even values only).
   * - ``flicker_detection_enabled``
     - bool
     - ``true``
     - Enable ambient light flicker detection (50/60 Hz mains).

Integration Time Formula
------------------------

The integration time per measurement cycle is::

    t_integration = (atime + 1) * (astep + 1) * 2.78 µs

With default values::

    t = (29 + 1) * (599 + 1) * 2.78 µs = 50.04 ms

In 18-channel mode, the sensor performs 3 sequential measurement cycles, so the total time for a complete reading is approximately **3 × t_integration**.

**Maximum useful publish rate:**

.. list-table::
   :header-rows: 1
   :widths: 20 20 20 20

   * - SMUX Mode
     - Cycles
     - Total Time (default)
     - Max Rate
   * - 6-channel
     - 1
     - ~50 ms
     - ~20 Hz
   * - 12-channel
     - 2
     - ~100 ms
     - ~10 Hz
   * - 18-channel
     - 3
     - ~150 ms
     - ~6.6 Hz

Setting ``publish_rate_hz`` higher than the sensor can deliver will result in duplicate readings.

Gain Selection Guide
--------------------

.. list-table::
   :header-rows: 1
   :widths: 15 30 55

   * - Gain
     - Use Case
     - Notes
   * - 1–4x
     - Direct sunlight, bright LEDs
     - Lowest sensitivity, avoids saturation in very bright conditions
   * - 8–64x
     - Indoor lighting, moderate brightness
     - Good balance of sensitivity and dynamic range
   * - 128–256x
     - Low-light conditions
     - Default setting, suitable for most indoor/lab use
   * - 512–2048x
     - Very dim light, spectroscopy applications
     - Maximum sensitivity, highest risk of saturation with ambient light

If ``analog_saturation`` or ``digital_saturation`` flags appear in the published data, reduce the gain or integration time.

Example Configurations
----------------------

**Fast scanning (6-channel, high rate):**

.. code-block:: yaml

    as7343_node:
      ros__parameters:
        gain: 64
        atime: 9
        astep: 299
        smux_mode: 6
        publish_rate_hz: 15.0

**High sensitivity (dark environment):**

.. code-block:: yaml

    as7343_node:
      ros__parameters:
        gain: 2048
        atime: 100
        astep: 999
        smux_mode: 18
        publish_rate_hz: 1.0

**Reflectance measurement (with LED):**

.. code-block:: yaml

    as7343_node:
      ros__parameters:
        gain: 128
        atime: 29
        astep: 599
        smux_mode: 18
        publish_rate_hz: 5.0
        led_enabled: true
        led_current_ma: 50
