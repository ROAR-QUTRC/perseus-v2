Data Interpretation
===================

Understanding the raw spectral channel data from the AS7343 and how to use it for various applications.

Raw ADC Counts
--------------

All 14 spectral channel values published in ``~/spectral_data`` are **raw 16-bit unsigned ADC counts** (0–65535). These values are proportional to the incident light intensity at each channel's wavelength, modulated by:

1. **Gain setting** — higher gain = higher counts for the same light level
2. **Integration time** — longer integration = higher counts
3. **Spectral response** — each channel has a different sensitivity curve (defined by its Gaussian-like filter)

To compare readings across different gain/integration settings, normalize the counts::

    normalized = raw_count / (gain * integration_time_ms)

Spectral Channel Map
--------------------

The 14 channels span from violet (405 nm) to near-infrared (855 nm):

.. code-block:: text

    UV   Violet   Blue   Cyan   Green   Yellow   Orange   Red   Deep Red   NIR
    ─────|───────|──────|──────|───────|────────|────────|─────|──────────|──────
         F1     F2  FZ  F3    F4     F5  FY    FXL     F6    F7     F8    NIR
         405    425 450 475   515    550 555   600     640   690    745   855 nm

**Narrow-band channels** (F1–F8, NIR): ~20–55 nm FWHM, best for identifying specific spectral features.

**Wide-band channels** (FZ, FY, FXL): ~55–100 nm FWHM, capture broader spectral energy. FY closely approximates the human photopic response curve.

**Clear channel** (VIS): Broadband visible light, useful as a reference for normalization.

Saturation
----------

When a channel receives too much light for the current gain/integration settings:

- **Analog saturation** (``analog_saturation = true``): The photodiode amplifier has clipped. The raw count may read near 65535. All channels may be affected.
- **Digital saturation** (``digital_saturation = true``): The digital accumulator has overflowed. Reduce integration time (``atime`` or ``astep``).

**Response:** Reduce ``gain`` or shorten integration time. The node logs a throttled warning when saturation is detected.

Common Applications
-------------------

Colour Measurement
^^^^^^^^^^^^^^^^^^

Use the narrow-band channels to reconstruct a rough spectral power distribution (SPD). The channels can be mapped to CIE colour matching functions to estimate:

- **Correlated Colour Temperature (CCT)** — ratio of blue to red channels
- **CRI (Colour Rendering Index)** — compare measured SPD against a reference illuminant

A simple CCT estimate using the blue/red ratio::

    ratio = fz_450nm / f6_640nm
    # Higher ratio = cooler (bluer) light, lower ratio = warmer (redder) light

Material Classification
^^^^^^^^^^^^^^^^^^^^^^^

Different materials have characteristic spectral reflectance signatures. By illuminating a sample with the on-board LED and measuring the reflected spectrum:

1. Enable the LED (``led_enabled: true``)
2. Record spectral readings of known reference materials
3. Compare unknown samples against the reference library

Each material produces a distinct pattern across the 14 channels — this "spectral fingerprint" can be used for classification.

Vegetation / NDVI
^^^^^^^^^^^^^^^^^^

The Near-Infrared (NIR, 855 nm) and Red (F6, 640 nm) channels enable a Normalized Difference Vegetation Index calculation::

    NDVI = (NIR_855nm - F6_640nm) / (NIR_855nm + F6_640nm)

- **NDVI > 0.6**: Dense, healthy vegetation
- **NDVI 0.2–0.6**: Moderate vegetation
- **NDVI < 0.2**: Bare soil, water, or non-vegetated surface

Light Source Identification
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Different light sources have distinct spectral signatures:

- **Incandescent**: Smooth curve, stronger in red/NIR
- **Fluorescent**: Spikes at specific wavelengths (especially 545 nm, 610 nm)
- **LED**: Narrow peak (typically 450 nm blue) + broad phosphor emission
- **Sunlight**: Relatively flat across all visible channels
- **Sodium vapour**: Strong peak around 590 nm (FXL channel)

Use the ``~/flicker_status`` topic to distinguish 50 Hz and 60 Hz mains-frequency lighting.

Flicker Detection
-----------------

The ``~/flicker_status`` topic reports ambient light flicker:

- **100 Hz detected**: 50 Hz mains frequency (common in Europe, Asia, Africa, Australia)
- **120 Hz detected**: 60 Hz mains frequency (common in North America, parts of South America and Asia)
- **0 Hz (none)**: DC lighting (LED drivers with good regulation), sunlight, or insufficient signal

This is useful for:

- Camera exposure synchronization (avoiding banding)
- Identifying power grid frequency in the local environment
- Detecting LED PWM dimming frequency

Data Quality Checks
-------------------

When processing spectral data, verify:

1. **``data_valid`` is true** — if ``false``, the measurement cycle did not complete
2. **No saturation flags** — saturated channels are clipped and unreliable
3. **Clear channel is non-zero** — a zero clear channel indicates no light reaching the sensor
4. **Channel ratios are stable** — if ratios between channels vary wildly between readings, check sensor mounting and stray light
