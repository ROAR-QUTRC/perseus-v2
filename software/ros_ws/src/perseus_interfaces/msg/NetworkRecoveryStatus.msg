builtin_interfaces/Time stamp

# State constants (published as uint8):
#   0 = MONITORING          — normal operation, network is healthy
#   1 = SIGNAL_DEGRADED     — signal below threshold but still connected
#   2 = SIGNAL_LEASH        — signal trend is degrading, robot paused as warning
#   3 = SIGNAL_LOST_WAITING — no connectivity, waiting for timeout before acting
#   4 = RECOVERING          — actively navigating to regain signal
#   5 = RECOVERY_SUCCEEDED  — signal reacquired after recovery
#   6 = RECOVERY_FAILED     — exhausted all recovery options without regaining signal

uint8 MONITORING = 0
uint8 SIGNAL_DEGRADED = 1
uint8 SIGNAL_LEASH = 2
uint8 SIGNAL_LOST_WAITING = 3
uint8 RECOVERING = 4
uint8 RECOVERY_SUCCEEDED = 5
uint8 RECOVERY_FAILED = 6

uint8 state

# Human-readable status description
string status_message

# Network metrics (updated every ping cycle)
int8 signal_rssi_dbm
float32 ping_latency_ms
bool connected

# Signal trend: STABLE=0, IMPROVING=1, DEGRADING=2
uint8 TREND_STABLE = 0
uint8 TREND_IMPROVING = 1
uint8 TREND_DEGRADING = 2

uint8 signal_trend
float32 signal_trend_rate

# Recovery progress
uint16 breadcrumbs_total
uint16 breadcrumbs_remaining
float64 seconds_since_last_connected

# Robot position when this message was generated
geometry_msgs/PoseStamped current_pose
