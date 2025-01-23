# Running the Gazebo Simulation

This readme file explains how to launch the Gazebo Simulation environment and control the rover.

There is a separate nix devshell for simulation which is accessed via:

```
nix develop .#simulation
```

To run the most basic version of the simulation you will need to run the following in **two separate terminals**.

## Terminal 1: Launch Simulation Environment

To create the simulation:

```
ros2 launch perseus_simulation perseus_sim.launch.py
```

## Terminal 2: Control Perseus via keyboard

To send control commands using the keyboard:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## Note on start first time

The lunar moonscape model is downloaded the first time you launch the simulation.

It can take a while to download from the gazebo server and there are
only warnings about missing world files to indicate this process is happening (no clear indication of progress of the download).

Therefore when you first launch, please have an internet connection and let gazebo operate for up to 20 minutes to complete the download. It can be useful to monitor your network connection whilst this is happening (e.g. with btop).
