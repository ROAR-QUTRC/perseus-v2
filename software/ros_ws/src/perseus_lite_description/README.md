# Perseus Lite Description

This package contains the URDF description files for the Perseus Lite robot, a 50% scaled version of the Perseus rover designed for operation in sandy environments.

## Robot Specifications

- **Scale**: 50% of original Perseus robot
- **Wheels**: 4 wheels with rocker suspension
- **Steering**: Skid/differential steering
- **Motors**: ST3215 continuous servos with 12-bit encoders (one per wheel)
- **Control**: Serial communication to servos

## File Structure

```none
perseus_lite_description/
├── urdf/                    # URDF/Xacro files
│   ├── perseus_lite.urdf.xacro      # Main robot description
│   ├── chassis.urdf.xacro           # Chassis and differential bar
│   ├── rocker.urdf.xacro            # Rocker suspension arms
│   ├── motor_wheel.urdf.xacro       # Motor and wheel assemblies
│   ├── wheel.urdf.xacro             # Individual wheel definitions
│   └── perseus_lite.materials.xacro # Visual materials
├── meshes/                  # 3D model files
├── ros2_control/           # ROS2 control configuration
│   └── perseus_lite.ros2_control.xacro
├── rviz/                   # RViz configuration files
└── launch/                 # Launch files (to be added)
```

## Usage

### View in RViz

```bash
ros2 launch perseus_lite_description view_robot.launch.py
```

### Generate URDF

```bash
xacro perseus_lite.urdf.xacro prefix:= > perseus_lite.urdf
```

## Servo ID Mapping

- Front Left Wheel: ID 1
- Front Right Wheel: ID 2
- Rear Left Wheel: ID 3
- Rear Right Wheel: ID 4

## Scaling Notes

All masses and dimensions are scaled from the original Perseus robot:

- Linear dimensions: 50% (0.5x)
- Masses: 50% (0.5x)
- Inertias: 25% (0.25x) due to square scaling factor

## Dependencies

- `urdf`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher_gui` (for testing)
- `rviz2` (for visualisation)
