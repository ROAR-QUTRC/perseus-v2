# hi-can tests

Unit tests for the hi-can serialization primitives and per-device parameter
groups, built with GoogleTest.

## What's covered

- `serializable_test.cpp` — the foundational primitives every device serializer
  is built on: `scaled_int32_t`, `scaled_int16_t`, `SimpleSerializable<T>`,
  `wrapped_value_t<T>`. Round-trip and golden-byte assertions. Also pins the
  wire layout for `bucket_controller::{speed_t, current_t, magnet_t}` as a
  worked example for new `SimpleSerializable<wrapped_value_t<T>>`-based
  device typedefs (e.g. arm end-effector PWM).
- `parameter_group_test.cpp` — end-to-end test for `VescParameterGroup`. Uses
  the `FifoCanInterface` mock to inject CAN frames and asserts the group's
  getters decoded as expected. Two `DISABLED_` tests document latent bugs in
  the VESC serialize/deserialize paths (see the comments in the file); remove
  the prefix when the underlying code is fixed.
- `esc_parameter_group_test.cpp` — `EscParameterGroup`. The
  `StartupTransmissionsRequestsLimits` test is the explicit regression catcher
  for the PR #436 scenario where the LIMITS request was silently commented
  out during a namespace-clash fix.
- `filter_test.cpp` — `filter_t::matches` and `PacketManager` dispatch with
  masked filters (e.g. `DEVICE_MASK`), so device tests that use partial
  filters have a reference for the expected behaviour.

Two helper headers live alongside the tests:

- `test_helpers.hpp` — `be32`, `be16`, `concat` for building golden CAN
  payloads.
- `mock_can_interface.hpp` — `FifoCanInterface`, a `FilteredCanInterface`
  backed by FIFO queues for inject/inspect.

## Adding a test for a new device

When you add a new `ParameterGroup` subclass, add an entry to
`parameter_group_test.cpp` (or a new `<device>_test.cpp` file if the device
warrants its own suite). The `VescParameterGroupTest` and
`EscParameterGroupTest` fixtures are both copy-paste templates.

1. Construct a `FifoCanInterface`, build a `PacketManager`, and `add_group`
   your group. Clear `bus.transmitted` in `SetUp` after `add_group` if the
   group has any zero-interval transmissions, otherwise the first
   `handle()` call will emit stray frames.
2. For each receive frame your device handles, write a test that queues a
   golden-byte frame on the FIFO, calls `packet_manager->handle_receive()`,
   and asserts the group's getter returns the expected value. **Source the
   golden bytes from your device datasheet, not by serializing through the
   code under test** — otherwise a bug in the serializer hides itself.
3. For each transmit frame, drive the group's setters, call
   `handle(false, /*force_transmission=*/true)`, and search the
   `transmitted` queue by address rather than indexing positionally — a
   group may emit multiple frames per cycle, and ordering is not part of
   the contract.
4. If you add a new `.cpp` file, **`git add` it** (staging is enough — no
   commit needed) before running `nix build`. The flake source filter uses
   `git ls-files`, so untracked test files are invisible to the build and
   `add_executable` will fail with "Cannot find source file". List the new
   path in `HI_CAN_TEST_SOURCES` in the parent `CMakeLists.txt`.

## Not yet covered

- **Timeout / timeout-recovery callbacks.** `PacketManager` reads
  `steady_clock::now()` directly inside `handle_receive`, so timeout tests
  would either need a clock abstraction injected into `PacketManager` or a
  `std::this_thread::sleep_for` (flaky, slow). Deferred until that refactor
  is done.

## Running

Tests run automatically via the nix build (`doCheck = true` in
`default.nix`):

```bash
nix build .#pkgs.hi-can
```

or by hand inside a build directory:

```bash
cd software/shared/hi-can
cmake -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```
