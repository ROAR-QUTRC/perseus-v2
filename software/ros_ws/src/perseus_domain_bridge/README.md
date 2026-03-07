# domain_bridge_throttled

A ROS 2 **C++ / ament_cmake** package that extends `ros2/domain_bridge` with
**per-topic Hz rate-limiting** when bridging topics across DDS domains.

---

## Why this exists

`domain_bridge` forwards messages 1:1 — whatever Hz the source publishes is
what arrives on the destination domain. There is no built-in way to throttle.

This package adds a `max_rate` field per topic in the YAML config. A 60 Hz
camera on domain 0 can be forwarded at 30 Hz to domain 1 with a one-line
change.

---

## Package structure

```
domain_bridge_throttled/
├── CMakeLists.txt
├── package.xml
├── include/domain_bridge_throttled/
│   └── throttled_bridge.hpp      # ThrottledDomainBridge, ThrottledGenericBridge
├── src/
│   ├── throttled_bridge.cpp      # Implementation
│   └── throttled_bridge_node.cpp # main()
├── launch/
│   ├── throttled_bridge.launch.py   # This package (throttled)
│   └── domain_bridge.launch.py      # Upstream domain_bridge (unthrottled)
└── config/
    └── bridge_config.yaml           # Example config
```

---

## Build

```bash
# Inside your ROS 2 workspace root
cp -r domain_bridge_throttled src/

rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select domain_bridge_throttled
source install/setup.bash
```

---

## Configuration

```yaml
# Global defaults
from_domain: 0
to_domain: 1

topics:
  /camera/image_raw:
    type: sensor_msgs/msg/Image
    max_rate: 30.0 # Hz — source at 60 Hz, bridge at 30 Hz
    qos: best_effort # best_effort | reliable
    # remap: /other/topic # rename topic in destination domain (optional)

  /odom:
    type: nav_msgs/msg/Odometry
    qos: reliable
    # no max_rate → full rate passthrough

  /special:
    type: sensor_msgs/msg/Imu
    from_domain: 5 # per-topic domain override
    to_domain: 2
    max_rate: 50.0
```

### All per-topic keys

| Key           | Required | Default        | Description                                |
| ------------- | -------- | -------------- | ------------------------------------------ |
| `type`        | ✅       | —              | ROS msg type e.g. `sensor_msgs/msg/Image`  |
| `max_rate`    | ❌       | `0`            | Max forward rate in Hz. `0` = unlimited    |
| `qos`         | ❌       | `best_effort`  | `best_effort` or `reliable`                |
| `remap`       | ❌       | same as source | Topic name on the destination domain       |
| `from_domain` | ❌       | global value   | Override source domain for this topic      |
| `to_domain`   | ❌       | global value   | Override destination domain for this topic |

---

## Running

**Throttled bridge:**

```bash
ros2 launch domain_bridge_throttled throttled_bridge.launch.py \
    config:=/path/to/bridge_config.yaml
```

Or directly:

```bash
ros2 run domain_bridge_throttled throttled_bridge /path/to/bridge_config.yaml
```

**Plain domain_bridge (no throttle):**

```bash
ros2 launch domain_bridge_throttled domain_bridge.launch.py \
    config:=/path/to/plain.yaml \
    from_domain:=0 \
    to_domain:=1
```

---

## How the throttle works

```
on each incoming message:
  now = steady_clock::now()
  if (now - last_published) >= (1.0 / max_rate):
      publish(msg)
      last_published = now
  else:
      drop msg
```

- Uses `std::chrono::steady_clock` (monotonic, no wall-clock jumps).
- Always forwards the **most recent** message — no stale buffering.
- Thread-safe per-topic mutex for `last_published`.
- Zero extra latency on the publish path.
- Falls back to full-rate forwarding when `max_rate` is `0` or omitted
  (the interval check is skipped entirely).

---

## Comparison

| Approach                      | Rate limit | Cross-domain | Any msg type |
| ----------------------------- | ---------- | ------------ | ------------ |
| `domain_bridge`               | ❌         | ✅           | ✅           |
| `topic_tools/throttle`        | ✅         | ❌           | ✅           |
| **`domain_bridge_throttled`** | ✅         | ✅           | ✅           |
