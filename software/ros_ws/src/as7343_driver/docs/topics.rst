Published Topics
================

The AS7343 node publishes the following topics. All topic names are relative to the node namespace (default: ``/as7343_node/``).

``~/spectral_data``
-------------------

**Type:** ``perseus_interfaces/msg/SpectralData``

**Rate:** Configured by ``publish_rate_hz`` parameter (default 5 Hz)

Contains the 14 unique spectral channel readings from the AS7343, plus sensor status. All channel values are raw 16-bit ADC counts (0–65535).

.. list-table:: Spectral Channel Fields
   :header-rows: 1
   :widths: 20 15 15 50

   * - Field
     - Wavelength
     - FWHM
     - Description
   * - ``f1_405nm``
     - 405 nm
     - ~30 nm
     - Violet
   * - ``f2_425nm``
     - 425 nm
     - ~22 nm
     - Violet-Blue
   * - ``fz_450nm``
     - 450 nm
     - ~55 nm
     - Blue (wide-band)
   * - ``f3_475nm``
     - 475 nm
     - ~30 nm
     - Blue-Cyan
   * - ``f4_515nm``
     - 515 nm
     - ~40 nm
     - Green-Cyan
   * - ``f5_550nm``
     - 550 nm
     - ~35 nm
     - Green
   * - ``fy_555nm``
     - 555 nm
     - ~100 nm
     - Yellow-Green (wide-band, photopic response)
   * - ``fxl_600nm``
     - 600 nm
     - ~80 nm
     - Orange (wide-band)
   * - ``f6_640nm``
     - 640 nm
     - ~50 nm
     - Red
   * - ``f7_690nm``
     - 690 nm
     - ~55 nm
     - Deep Red
   * - ``f8_745nm``
     - 745 nm
     - ~50 nm
     - Near-IR
   * - ``nir_855nm``
     - 855 nm
     - ~20 nm
     - Near-IR
   * - ``vis_clear``
     - Broadband
     - Full visible
     - Clear/visible channel (averaged across 3 measurement cycles)
   * - ``fd_flicker``
     - N/A
     - N/A
     - Flicker detection channel (averaged across 3 measurement cycles)

.. list-table:: Status Fields
   :header-rows: 1
   :widths: 25 75

   * - Field
     - Description
   * - ``analog_saturation``
     - ``true`` if the analog ADC has saturated. Reduce gain or integration time.
   * - ``digital_saturation``
     - ``true`` if the digital accumulator has saturated. Reduce integration time.
   * - ``data_valid``
     - ``true`` if the AVALID flag was set, indicating a complete measurement cycle.
   * - ``integration_time_ms``
     - The current integration time per cycle in milliseconds.
   * - ``gain``
     - The current gain multiplier (1–2048).


``~/flicker_status``
--------------------

**Type:** ``perseus_interfaces/msg/FlickerStatus``

**Rate:** Same as ``publish_rate_hz`` (only published if ``flicker_detection_enabled`` is ``true``)

Reports ambient light flicker detection. Useful for identifying artificial lighting frequency.

.. list-table:: Fields
   :header-rows: 1
   :widths: 30 70

   * - Field
     - Description
   * - ``detected_frequency_hz``
     - Detected flicker frequency: ``0`` (none), ``100`` (50 Hz mains), or ``120`` (60 Hz mains)
   * - ``hz_100_valid``
     - ``true`` if the 100 Hz measurement is valid
   * - ``hz_120_valid``
     - ``true`` if the 120 Hz measurement is valid
   * - ``hz_100_detected``
     - ``true`` if 100 Hz flicker was detected
   * - ``hz_120_detected``
     - ``true`` if 120 Hz flicker was detected
   * - ``fd_saturation``
     - ``true`` if the flicker detection channel is saturated
   * - ``fd_valid``
     - ``true`` if the flicker measurement is complete and valid


``~/integration_time_ms``
-------------------------

**Type:** ``std_msgs/msg/Float64``

**Rate:** Same as ``publish_rate_hz``

Publishes the current integration time per measurement cycle in milliseconds. This is a convenience topic for monitoring — the same value is also included in the ``spectral_data`` message.
