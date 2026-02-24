Hardware Setup
==============

Supported Hardware
------------------

- **Sensor:** ams-OSRAM AS7343 14-channel multi-spectral sensor
- **Breakout boards:** SparkFun AS7343 Qwiic, Adafruit AS7343, Pimoroni AS7343
- **Host:** Raspberry Pi 4 Model B, Raspberry Pi 5

The AS7343 communicates over I2C at a default address of ``0x39`` and supports I2C Fast Mode (400 kHz).

Wiring
------

The sensor uses a standard I2C connection with 4 wires:

.. list-table::
   :header-rows: 1
   :widths: 20 20 30

   * - Sensor Pin
     - Raspberry Pi Pin
     - Notes
   * - VCC / 3V3
     - Pin 1 (3.3V)
     - Most breakout boards have an onboard regulator and accept 3.3V or 5V
   * - GND
     - Pin 6 (GND)
     - Any ground pin
   * - SDA
     - Pin 3 (GPIO 2 / SDA1)
     - I2C data line
   * - SCL
     - Pin 5 (GPIO 3 / SCL1)
     - I2C clock line

If using a Qwiic/STEMMA QT connector, simply connect the cable between the sensor breakout and a Qwiic HAT/pHAT on the Pi.

Enabling I2C on Raspberry Pi
----------------------------

Raspberry Pi 4
^^^^^^^^^^^^^^

1. Enable I2C via ``raspi-config``::

    sudo raspi-config
    # Navigate to: Interface Options -> I2C -> Enable

2. Or add to ``/boot/config.txt``::

    dtparam=i2c_arm=on

3. Reboot and verify::

    sudo reboot
    ls /dev/i2c-*
    # Should show /dev/i2c-1

Raspberry Pi 5
^^^^^^^^^^^^^^

1. Enable I2C via ``raspi-config`` (same as Pi 4), or add to ``/boot/firmware/config.txt``::

    dtparam=i2c_arm=on

2. Reboot and verify::

    sudo reboot
    ls /dev/i2c-*
    # Should show /dev/i2c-1

Verifying the Sensor
--------------------

Install I2C tools and scan the bus::

    sudo apt-get install -y i2c-tools
    i2cdetect -y 1

You should see ``39`` appear in the scan output, confirming the AS7343 is detected::

         0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:                         -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- 39 -- -- -- -- -- --
    ...

I2C Permissions
---------------

By default, ``/dev/i2c-1`` requires root access. To allow your ROS 2 user to access I2C without sudo:

1. Add your user to the ``i2c`` group::

    sudo usermod -aG i2c $USER

2. Create a udev rule (if the ``i2c`` group doesn't exist)::

    echo 'SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"' | sudo tee /etc/udev/rules.d/99-i2c.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger

3. Log out and back in for group changes to take effect.

I2C Bus Speed
-------------

The AS7343 supports I2C Fast Mode (400 kHz). The default Raspberry Pi I2C speed is 100 kHz. To increase it:

**Raspberry Pi 4** — edit ``/boot/config.txt``::

    dtparam=i2c_arm=on,i2c_arm_baudrate=400000

**Raspberry Pi 5** — edit ``/boot/firmware/config.txt``::

    dtparam=i2c_arm=on,i2c_arm_baudrate=400000

Reboot after changing. The higher bus speed reduces I2C transaction time, which is beneficial when reading all 18 data registers in 18-channel mode.
