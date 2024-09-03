# Rover Bringup

The purpose of this document is to bring up the rover from scratch.

Clone this repo into the onboard computer and on some kind of control device. In most cases the GCS will be the controlling
device, which is going to be slightly different than using a gamepad.

## Installing Requirements (All machines)

ros2 humble is a central requirement for running the rover, following the instructions for
- ubuntu 22.04 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- ubuntu 20.04,https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html
- windows https://robostack.github.io/index.html

(ubuntu 22.04 is recommended)

Once that is installed you can go ahead and build the source for for the required environments

## Build Source Code (All machines)

Change directories into `Artemis_ws/Software/rover_ws` and build the source files with the following commands

```bash
cd Artemis_ws/Software/rosver_ws
colcon build
```

if you don't want to build the whole workspace we do have metapackages you can target that will only build the required 
packages. They are `rover` and `gcs`, and can build built separatly with 

```bash
colcon build --packages-up-to rover # for rover
colcon build --packages-up-to gcs   # for gcs
```
Once those are build you'll need to source the newly build workspace. This is done with

```bash
source ./install/setup.bash
source ./install/setup.zsh
```

(Ubuntu uses bash by default, only macos uses zsh by default)


## Run the rover (Rover)

Making sure to source the directly

```bash
ros2 launch rover rover.launch.py
```

## Run the GCS (GCS)

You'll run two different nodes, depending on if you're using a hotas warthog or a gamepad (most gamepads with be compatible)

gamepad

```bash
ros2 run gcs_controllers generic_controller_node
```

Using the gamepad will only allow you to control drive with the left stick

hotas warthog

```bash
ros2 run gcs_controllers hotas_warthog_node
```

See documentation in ondrive for controlling the rover with the hotas warthog.

## Warnings

- Make sure to sanity check the controllers with `ros2 topic echo /rover_drive/twist` with the motors off, especially when using a new device
- The playstation3 controllers  can output garbage controls if they are turned off but plugged in.

that's it, you should be good to go.