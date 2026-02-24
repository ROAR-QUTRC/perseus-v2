# AS7343 flicker detection status
# Reports detected ambient light flicker frequency

std_msgs/Header header

# Detected flicker frequency in Hz (0 = none, 100 = 50Hz mains, 120 = 60Hz mains)
uint16 detected_frequency_hz

# Validity flags
bool hz_100_valid    # 100Hz measurement valid
bool hz_120_valid    # 120Hz measurement valid
bool hz_100_detected # 100Hz flicker detected
bool hz_120_detected # 120Hz flicker detected

# Raw flicker detection status
bool fd_saturation   # Flicker detection channel saturated
bool fd_valid        # Flicker detection measurement complete and valid
