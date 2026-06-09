# Error Log

Defects found and fixed in this codebase, recorded per the error-tracking
workflow. Review the **Prevention rules** before modifying the listed files.

### Ineffective serial read timeout in ST3215 hardware interface — 2026-06-10

- **Severity:** Critical
- **Category:** API Misuse
- **File(s):** `software/ros_ws/src/perseus_lite_hardware/src/st3215_system.cpp`
- **Pattern:** Constructing a `boost::asio::steady_timer` next to a blocking I/O call without ever `async_wait`ing on it — the timer object does nothing unless it is wired into the same `io_context` run loop as an async operation.
- **Root cause:** The communication thread created a timer and then called the blocking `serial_port.read_some()`, assuming the timer's existence imposed a deadline. A wedged servo or unplugged USB adapter would block the thread forever.
- **Fix applied:** Added `read_with_timeout()` which pairs `async_read_some` with a deadline timer on the shared `io_context` and runs it to completion; timeout cancels the read and is reported as `operation_aborted`. The never-started `_io_thread` and unused async machinery were removed.
- **Prevention rule:** Never mix a constructed-but-unawaited asio timer with blocking I/O. If a timeout is required, both the I/O and the timer must be async operations on the same `io_context`, with each handler cancelling the other.

### Servo temperature state read the voltage register — 2026-06-10

- **Severity:** High
- **Category:** Logic
- **File(s):** `software/ros_ws/src/perseus_lite_hardware/src/st3215_system.cpp`, `software/ros_ws/src/perseus_lite_hardware/include/st3215_protocol.hpp`
- **Pattern:** Hardcoded byte-offset constants for a wire format that drift from the device register map (off-by-one across a multi-byte field).
- **Root cause:** The status read covers registers 0x38–0x3F (position 2, speed 2, load 2, voltage 1, temperature 1). With the error byte at payload offset 0, temperature lands at offset 8, but `_TEMPERATURE_BYTE_INDEX` was 7 — the voltage register — so the exported temperature state reported bus voltage in 0.1 V units as °C.
- **Fix applied:** Packet parsing extracted into `protocol::extract_packets()` / `protocol::parse_status_payload()` with the correct offsets and a regression unit test (`ParseStatusPayloadReadsTemperatureNotVoltage`).
- **Prevention rule:** When parsing register-block reads, derive byte offsets from the register table (start register + sizes), write them next to a comment naming the registers, and pin them with a unit test using distinct sentinel values in adjacent fields.

### Serial request/response transaction not atomic — 2026-06-10

- **Severity:** High
- **Category:** Concurrency
- **File(s):** `software/ros_ws/src/perseus_lite_hardware/src/st3215_system.cpp`
- **Pattern:** Locking a shared bus only for the send half of a request/response exchange, allowing another thread's write to interleave between request and reply.
- **Root cause:** The communication thread released `_serial_mutex` after sending the status request and read the response unlocked, while the controller's `write()` could acquire the mutex and transmit velocity packets mid-exchange, corrupting the reply stream.
- **Fix applied:** The mutex is now held across the complete send → wait → read transaction.
- **Prevention rule:** On a half-duplex serial bus, the unit of mutual exclusion is the whole transaction (request plus response), never just the write.

### CI main-branch gate referenced the upstream repository — 2026-06-10

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `.github/workflows/all.yaml`, `README.md`
- **Pattern:** Fork retains `github.repository == '<upstream>/<repo>'` guards (and badges/URLs) from the parent project, silently disabling main-branch automation.
- **Root cause:** `is-main` checked for `ROAR-QUTRC/perseus-v2`, which is never true on `DingoOz/perseus-lite`, so jobs gated on it could never run; the README badge pointed at the upstream repo as well.
- **Fix applied:** Updated the repository check, gated the docs-website deploy on a `DOCS_DEPLOY_KEY` secret existing, and fixed the README title/badge.
- **Prevention rule:** After forking, grep workflows and docs for the upstream `owner/repo` string; every `github.repository ==` comparison must name the fork.

### Launch arguments declared but never forwarded to the included launch file — 2026-06-10

- **Severity:** High
- **Category:** Configuration
- **File(s):** `software/ros_ws/src/perseus_lite_missions/launch/mission_zero.launch.py`, `software/ros_ws/src/perseus_lite_simulation/launch/perseus_sim.launch.py`
- **Pattern:** `DeclareLaunchArgument` in a parent launch file without passing the value through `launch_arguments` of the `IncludeLaunchDescription` chain — the argument parses fine but has no effect.
- **Root cause:** `mission_zero.launch.py` declared `gz_world:=selene_base.sdf` and `initial_pose_*`, but `perseus_sim.launch.py` neither accepted nor forwarded them, so Mission Zero silently launched the default arc world at the default spawn pose.
- **Fix applied:** `perseus_sim.launch.py` now declares and forwards `gz_world` and `initial_pose_*` to `gazebo.launch.py`, and `mission_zero.launch.py` passes them down.
- **Prevention rule:** Every declared launch argument must appear either in a `Node`/process definition or in a child's `launch_arguments`; after adding one, verify it reaches the leaf with `ros2 launch <pkg> <file> --show-args` plus a grep of the include chain.
