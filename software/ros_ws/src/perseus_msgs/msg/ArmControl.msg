std_msgs/Header header

# Position of the actuators in [rad] for angular actuators and [m] for linear actuators
float64[] position

# Velocities of the actuators in [rad/s] for angular actuators and [m/s] for linear actuators
float64[] velocity

# Everything that does not fit the above, normalized between [0.0 ... 1.0]
# For PWM servos: duty cycle percentage (0.0 = 0%, 1.0 = 100%)
float64[] normalized

uint16 time_ms
uint8 acceleration
