# Arm Hi-CAN Protocol

The arm control board is the CAN gateway for all six joints:

1. Shoulder pan
2. Shoulder tilt
3. Elbow
4. Wrist pitch
5. Wrist roll
6. Tool

## Messages

- `PROBE`: Check whether a joint is communicating.
- `JOINT_STATE`: Reports ready, enabled, zeroed, and fault state.
- `ZERO_POSITION`: Request joint zeroing.
- `SET_ENABLED`: Enable or disable a joint.
- `COMMAND`: Send position, speed, and acceleration.
- `STATUS_1`: Position, speed, and load.
- `STATUS_2`: Voltage, temperature, current, and movement state.

Status is split into two messages to remain within the 8-byte CAN payload limit.
Position and speed use milliradians and milliradians per second on CAN.

## Startup

1. Register `ControlBoardParameterGroup`; this probes every joint.
2. Wait for `JOINT_STATE` responses.
3. Report missing, unready, or faulted joints.
4. Send `ZERO_POSITION` to ready joints.
5. Wait until each responding joint reports `ZEROED`.
6. Enable and command only ready, zeroed, fault-free joints.

A failed joint is skipped. Other available joints may continue moving.

## Responsibilities

- **Hi-CAN:** Addresses, payloads, discovery state, status storage, and packet helpers.
- **Control-board firmware:** Respond to probes, zero joints, execute commands, and publish state/status.
- **Arm driver:** Run startup timeouts, report errors, and skip unavailable joints.
- **Arm controller:** Produce arm commands; it does not manage CAN discovery.

Use typed `joint_id` values with `get_status(joint_id)` rather than raw array indexes.

## ROS topics

- `/arm/can/control`: `Actuators`, ordered as the joint list above.
- `/arm/can/positions`: six positions in radians.
- `/arm/can/status`: 13 fields per joint: ID, communication, lifecycle flags,
  fault, motion status, and electrical status.
