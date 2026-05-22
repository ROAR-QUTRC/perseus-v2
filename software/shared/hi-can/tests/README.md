# hi-can tests

Unit tests for the hi-can serialization primitives and per-device parameter
groups, built with GoogleTest.

## What's covered

- `serializable_test.cpp` — the foundational primitives every device serializer
  is built on: `scaled_int32_t`, `scaled_int16_t`, `SimpleSerializable<T>`,
  `wrapped_value_t<T>`. Round-trip and golden-byte assertions.
- `parameter_group_test.cpp` — end-to-end test for one device's
  `ParameterGroup` (VESC drive ESC). Uses a `FifoCanInterface` mock to inject
  CAN frames and asserts the group's getters return the expected decoded
  values.

## Adding a test for a new device

When you add a new `ParameterGroup` subclass, add an entry to
`parameter_group_test.cpp` (or a new `<device>_test.cpp` file if the device
warrants its own suite):

1. Construct a `FifoCanInterface`, build a `PacketManager`, and `add_group`
   your group. The `VescParameterGroupTest` fixture is a copy-paste starting
   point.
2. For each receive frame your device handles, write a test that queues a
   golden-byte frame on the FIFO, calls `packet_manager->handle()`, and
   asserts the group's getter returns the expected value. **Source the golden
   bytes from your device datasheet, not by serializing through the code under
   test** — otherwise a bug in the serializer hides itself.
3. For each transmit frame, drive the group's setters, call
   `handle(false, /*force_transmission=*/true)`, and assert the expected frame
   ended up in the FIFO's `transmitted` queue.
4. If you add a new `.cpp` file, list it in `HI_CAN_TEST_SOURCES` in the
   parent `CMakeLists.txt`.

## Running

Tests run automatically via the nix build (`doCheck = true` in `default.nix`)
or by hand inside a build directory:

```bash
cd software/shared/hi-can
cmake -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```
