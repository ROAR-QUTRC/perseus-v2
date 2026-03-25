# AS7343 14-channel multi-spectral sensor data
# All channel values are raw 16-bit ADC counts

std_msgs/Header header

# Violet/Blue channels
uint16 f1_405nm    # Violet (405nm, FWHM ~30nm)
uint16 f2_425nm    # Violet-Blue (425nm, FWHM ~22nm)
uint16 fz_450nm    # Blue wide-band (450nm, FWHM ~55nm)
uint16 f3_475nm    # Blue-Cyan (475nm, FWHM ~30nm)

# Green channels
uint16 f4_515nm    # Green-Cyan (515nm, FWHM ~40nm)
uint16 f5_550nm    # Green (550nm, FWHM ~35nm)
uint16 fy_555nm    # Yellow-Green wide-band (555nm, FWHM ~100nm)

# Red/Orange channels
uint16 fxl_600nm   # Orange wide-band (600nm, FWHM ~80nm)
uint16 f6_640nm    # Red (640nm, FWHM ~50nm)
uint16 f7_690nm    # Deep Red (690nm, FWHM ~55nm)

# Near-IR channels
uint16 f8_745nm    # Near-IR (745nm, FWHM ~50nm)
uint16 nir_855nm   # Near-IR (855nm, FWHM ~20nm)

# Broadband / auxiliary
uint16 vis_clear   # Visible broadband (clear channel)
uint16 fd_flicker  # Flicker detection channel

# Sensor status
bool analog_saturation   # True if analog saturation detected
bool digital_saturation  # True if digital saturation detected
bool data_valid          # True if AVALID flag was set

# Current sensor configuration
float64 integration_time_ms  # Actual integration time in milliseconds
uint16 gain                  # Current gain multiplier (e.g. 1, 2, 4, ... 2048)
