# perseus-arm-teleop

Terminal based Arm Teleop for Perseus

## To build

```
mkdir build
```

```
cmake ..
make
```

The run the binaries found there.

## TODO

- Handle rollover of values (joint #2 really needs this)
- Add yaml config file loading
- Add PID to movements to remove jerkiness
- Add ability to include 24 servos (second leader and second follower)
- Colour code torque values (mostly for gripper)
- Add diagnostic info
