# Running the Gazebo Simulation

To run the most basic version of the simulation you will need to run the following in two separate termainals:

## Terminal 1

To create the simulation:

```
ros2 launch perseus_simulation perseus_sim.launch.py
```

## Terminal 2

To send control commands using the keyboard:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```
