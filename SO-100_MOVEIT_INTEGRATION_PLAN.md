# SO-100 Arm MoveIt! 2 Integration Plan

**Project:** Perseus-lite SO-100 Robotic Arm Integration
**Goal:** Full ROS2 + ros2_control + MoveIt! 2 integration for SO-100 6-DOF arm
**Hardware:** SO-100 arm with ST3215 servos
**Base Branch:** main
**Feature Branch:** feat/perseus-lite-so100-arm

---

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Phase 1: Branch Setup & Dependencies](#phase-1-branch-setup--dependencies)
- [Phase 2: Extend ST3215 Hardware Interface](#phase-2-extend-st3215-hardware-interface-for-position-control)
- [Phase 3: SO-100 Arm Description Package](#phase-3-so-100-arm-description-package)
- [Phase 4: MoveIt! 2 Configuration](#phase-4-moveit-2-configuration)
- [Phase 5: Controller Configuration](#phase-5-controller-configuration)
- [Phase 6: Example Applications & Testing](#phase-6-example-applications--testing)
- [Phase 7: Integration Testing & Tuning](#phase-7-integration-testing--tuning)
- [Technical Reference](#technical-reference)
- [Success Criteria](#success-criteria)

---

## Overview

This plan integrates the SO-100 6-DOF robotic arm with MoveIt! 2, building on the existing Perseus-lite ST3215 hardware interface. The integration will:

- Extend the existing `perseus_lite_hardware` package to support position commands
- Create robot description (URDF) and MoveIt configuration packages
- Implement trajectory control using `joint_trajectory_controller`
- Provide example applications demonstrating manipulation capabilities

**Key Architectural Decision:** Reuse existing `perseus_lite_hardware/ST3215SystemHardware` rather than creating new hardware interface. Only needs position command support added.

---

## Prerequisites

### Required Information

- [ ] Path to SO-100 URDF/xacro file with 6 joints defined
- [ ] SO-100 arm mesh files (STL/DAE format) in visual/ and collision/ folders
- [ ] Physical specifications: link lengths, joint limits, DH parameters
- [ ] Joint calibration data (min/max servo positions for each joint)

### Software Dependencies

Check these are installed in your ROS 2 workspace:

```bash
# MoveIt 2 packages
sudo apt install ros-${ROS_DISTRO}-moveit
sudo apt install ros-${ROS_DISTRO}-moveit-setup-assistant
sudo apt install ros-${ROS_DISTRO}-moveit-planners-ompl
sudo apt install ros-${ROS_DISTRO}-moveit-simple-controller-manager

# ros2_control controllers
sudo apt install ros-${ROS_DISTRO}-joint-trajectory-controller
sudo apt install ros-${ROS_DISTRO}-controller-manager
sudo apt install ros-${ROS_DISTRO}-robot-state-publisher

# Additional tools
sudo apt install ros-${ROS_DISTRO}-rviz2
sudo apt install ros-${ROS_DISTRO}-rqt-joint-trajectory-controller
```

### Verify Perseus-lite Packages

Ensure these packages are available (should be merged into main):

- [ ] `perseus_lite_hardware` - ST3215 hardware interface
- [ ] `perseus_lite_description` - Base robot description

```bash
# Check if packages exist
ls software/ros_ws/src/ | grep perseus_lite

# If not present, merge from feature branches first
```

---

## Phase 1: Branch Setup & Dependencies

### 1.1 Create Feature Branch

```bash
cd /home/dingo/perseus-v2

# Ensure you're on main and up to date
git checkout main
git pull origin main

# Create new feature branch for SO-100 arm
git checkout -b feat/perseus-lite-so100-arm

# Verify branch
git branch --show-current
```

### 1.2 Verify Build System

```bash
cd software/ros_ws

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build existing packages
colcon build --packages-select perseus_lite_hardware perseus_lite_description

# Verify build succeeded
echo $?  # Should output 0

# Source workspace
source install/setup.bash
```

### 1.3 Test Existing ST3215 Hardware Interface

```bash
# Check plugin is registered
ros2 pkg list | grep perseus_lite_hardware

# Verify plugin export
ros2 plugin list | grep ST3215

# Expected output: perseus_lite_hardware/ST3215SystemHardware
```

**Checkpoint:** Branch created, perseus_lite packages build successfully.

---

## Phase 2: Extend ST3215 Hardware Interface for Position Control

**Objective:** Add position command interface to existing velocity-only ST3215SystemHardware.

### 2.1 Current State Analysis

The existing implementation in `perseus_lite_hardware` provides:

- **Command Interface:** Velocity only (`HW_IF_VELOCITY`)
- **State Interfaces:** Position, Velocity, Temperature
- **Control Registers:** Uses GOAL_SPEED (0x2E) for velocity commands

**What we need to add:**

- **Command Interface:** Position (`HW_IF_POSITION`)
- **Control Register:** GOAL_POSITION (0x2A) for position commands
- **Calibration:** Load min/max servo positions, map radians ↔ servo raw values (0-4095)

### 2.2 Modify Header File

**File:** `software/ros_ws/src/perseus_lite_hardware/include/perseus_lite_hardware/st3215_system.hpp`

Add to class private members:

```cpp
// Position command storage (add alongside _command_speeds)
std::vector<double> _command_positions;

// Calibration data for position mapping
std::vector<double> _min_position_rad;   // Min joint position in radians
std::vector<double> _max_position_rad;   // Max joint position in radians
std::vector<uint16_t> _min_position_raw; // Min servo position (0-4095)
std::vector<uint16_t> _max_position_raw; // Max servo position (0-4095)

// Control mode parameter
std::string _control_mode;  // "position" or "velocity"

// Helper methods
uint16_t radians_to_servo_position(double radians, size_t joint_index);
double servo_position_to_radians(uint16_t servo_pos, size_t joint_index);
void load_calibration_data();
```

### 2.3 Modify Implementation File

**File:** `software/ros_ws/src/perseus_lite_hardware/src/st3215_system.cpp`

#### 2.3.1 Initialize New Members in `on_init()`

```cpp
hardware_interface::CallbackReturn ST3215SystemHardware::on_init(
    const hardware_interface::HardwareInfo& info)
{
    // ... existing initialization code ...

    const auto& joints = info.joints;
    const size_t num_joints = joints.size();

    // Initialize position command storage
    _command_positions.resize(num_joints, 0.0);
    _min_position_rad.resize(num_joints, 0.0);
    _max_position_rad.resize(num_joints, 0.0);
    _min_position_raw.resize(num_joints, 0);
    _max_position_raw.resize(num_joints, 4095);

    // Read control mode parameter (default to position for arm control)
    _control_mode = info.hardware_parameters.count("control_mode") > 0
        ? info.hardware_parameters.at("control_mode")
        : "position";

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                "ST3215 control mode: %s", _control_mode.c_str());

    // ... rest of existing code ...

    return hardware_interface::CallbackReturn::SUCCESS;
}
```

#### 2.3.2 Load Calibration Data in `on_configure()`

```cpp
hardware_interface::CallbackReturn ST3215SystemHardware::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    // ... existing serial port setup code ...

    // Load calibration data if available
    load_calibration_data();

    // ... rest of existing code ...
}

void ST3215SystemHardware::load_calibration_data()
{
    const auto& info = get_hardware_info();

    // Check for calibration file parameter
    if (info.hardware_parameters.count("calibration_file") > 0)
    {
        std::string calib_file = info.hardware_parameters.at("calibration_file");
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                    "Loading calibration from: %s", calib_file.c_str());

        // TODO: Parse YAML file and populate calibration vectors
        // For now, use per-joint parameters as fallback
    }

    // Load per-joint calibration parameters
    for (size_t i = 0; i < info.joints.size(); i++)
    {
        const auto& joint = info.joints[i];

        // Min position (default: 0 radians = 0 raw)
        if (joint.parameters.count("min_position_rad") > 0)
            _min_position_rad[i] = std::stod(joint.parameters.at("min_position_rad"));

        if (joint.parameters.count("max_position_rad") > 0)
            _max_position_rad[i] = std::stod(joint.parameters.at("max_position_rad"));

        if (joint.parameters.count("min_position_raw") > 0)
            _min_position_raw[i] = std::stoi(joint.parameters.at("min_position_raw"));

        if (joint.parameters.count("max_position_raw") > 0)
            _max_position_raw[i] = std::stoi(joint.parameters.at("max_position_raw"));

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                     "Joint %zu calibration: rad[%.3f, %.3f] raw[%u, %u]",
                     i, _min_position_rad[i], _max_position_rad[i],
                     _min_position_raw[i], _max_position_raw[i]);
    }
}
```

#### 2.3.3 Export Position Command Interface

```cpp
std::vector<hardware_interface::CommandInterface>
ST3215SystemHardware::export_command_interfaces()
{
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exporting command interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    const auto& info = get_hardware_info();

    for (size_t i = 0; i < info.joints.size(); i++)
    {
        if (_control_mode == "position")
        {
            // Export position command interface
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &_command_positions[i]));
        }
        else if (_control_mode == "velocity")
        {
            // Export velocity command interface (existing)
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &_command_speeds[i]));
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                        "Unknown control mode: %s", _control_mode.c_str());
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                 "Exported %zu command interfaces", command_interfaces.size());
    return command_interfaces;
}
```

#### 2.3.4 Implement Position Command in `write()`

```cpp
hardware_interface::return_type ST3215SystemHardware::write(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    // ... existing code ...

    if (_control_mode == "position")
    {
        // Position control mode - write GOAL_POSITION register
        for (size_t i = 0; i < _servo_ids.size(); i++)
        {
            // Convert radians to servo raw position (0-4095)
            uint16_t goal_pos = radians_to_servo_position(_command_positions[i], i);

            // Write to GOAL_POSITION register (0x2A)
            write_servo_register(_servo_ids[i], 0x2A, goal_pos);

            RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger(LOGGER_NAME),
                                  *_clock, 1000,  // Log every 1 second
                                  "Servo %u: commanded %.3f rad -> %u raw",
                                  _servo_ids[i], _command_positions[i], goal_pos);
        }
    }
    else if (_control_mode == "velocity")
    {
        // Velocity control mode (existing implementation)
        for (size_t i = 0; i < _servo_ids.size(); i++)
        {
            // ... existing velocity control code ...
        }
    }

    return hardware_interface::return_type::OK;
}
```

#### 2.3.5 Implement Helper Methods

```cpp
uint16_t ST3215SystemHardware::radians_to_servo_position(
    double radians, size_t joint_index)
{
    // Clamp to joint limits
    double clamped = std::clamp(radians,
                                _min_position_rad[joint_index],
                                _max_position_rad[joint_index]);

    // Linear mapping: radians -> servo raw position
    double range_rad = _max_position_rad[joint_index] - _min_position_rad[joint_index];
    double range_raw = _max_position_raw[joint_index] - _min_position_raw[joint_index];

    double normalized = (clamped - _min_position_rad[joint_index]) / range_rad;
    uint16_t servo_pos = _min_position_raw[joint_index] +
                         static_cast<uint16_t>(normalized * range_raw);

    // Clamp to servo hardware limits
    servo_pos = std::clamp(servo_pos, uint16_t(0), uint16_t(4095));

    return servo_pos;
}

double ST3215SystemHardware::servo_position_to_radians(
    uint16_t servo_pos, size_t joint_index)
{
    // Linear mapping: servo raw position -> radians
    double range_raw = _max_position_raw[joint_index] - _min_position_raw[joint_index];
    double range_rad = _max_position_rad[joint_index] - _min_position_rad[joint_index];

    double normalized = static_cast<double>(servo_pos - _min_position_raw[joint_index])
                        / range_raw;
    double radians = _min_position_rad[joint_index] + (normalized * range_rad);

    return radians;
}
```

### 2.4 Add ST3215 Position Write Function

You'll need to add a function to write to the GOAL_POSITION register. Check if this exists in the current implementation:

```cpp
void ST3215SystemHardware::write_servo_register(
    uint8_t servo_id, uint8_t reg_addr, uint16_t value)
{
    // Build ST3215 write packet
    std::vector<uint8_t> packet;
    packet.push_back(0xFF);  // Header 1
    packet.push_back(0xFF);  // Header 2
    packet.push_back(servo_id);
    packet.push_back(5);     // Length (instruction + addr + 2 bytes data + checksum)
    packet.push_back(0x03);  // WRITE instruction
    packet.push_back(reg_addr);
    packet.push_back(value & 0xFF);        // Low byte
    packet.push_back((value >> 8) & 0xFF); // High byte

    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 2; i < packet.size(); i++)
        checksum += packet[i];
    packet.push_back(~checksum);  // Bitwise NOT

    // Send via serial port
    boost::asio::write(_serial_port, boost::asio::buffer(packet));
}
```

### 2.5 Build and Test

```bash
cd /home/dingo/perseus-v2/software/ros_ws

# Build modified package
colcon build --packages-select perseus_lite_hardware

# Source
source install/setup.bash

# Verify plugin still loads
ros2 plugin list | grep ST3215
```

**Checkpoint:** ST3215SystemHardware now supports position commands with calibration mapping.

---

## Phase 3: SO-100 Arm Description Package

**Objective:** Create ROS 2 package with SO-100 URDF, meshes, and ros2_control configuration.

### 3.1 Create Package

```bash
cd /home/dingo/perseus-v2/software/ros_ws/src

# Create package
ros2 pkg create perseus_lite_arm_description \
    --build-type ament_cmake \
    --dependencies urdf xacro

cd perseus_lite_arm_description
```

### 3.2 Create Directory Structure

```bash
mkdir -p urdf meshes/visual meshes/collision config launch rviz
```

### 3.3 Copy Your SO-100 URDF

```bash
# Copy your existing SO-100 URDF file
cp /path/to/your/so100_arm.urdf.xacro urdf/

# Copy mesh files
cp /path/to/your/meshes/visual/* meshes/visual/
cp /path/to/your/meshes/collision/* meshes/collision/
```

### 3.4 Create ros2_control Configuration

**File:** `urdf/so100_arm.ros2_control.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:macro name="so100_arm_ros2_control" params="
    name
    serial_port:=/dev/ttyUSB0
    baud_rate:=115200
    prefix:=''
    use_mock_hardware:=false
    mock_sensor_commands:=false">

    <ros2_control name="${name}" type="system">

      <hardware>
        <xacro:if value="${use_mock_hardware}">
          <plugin>mock_components/GenericSystem</plugin>
          <param name="mock_sensor_commands">${mock_sensor_commands}</param>
          <param name="state_following_offset">0.0</param>
        </xacro:if>
        <xacro:unless value="${use_mock_hardware}">
          <plugin>perseus_lite_hardware/ST3215SystemHardware</plugin>
          <param name="serial_port">${serial_port}</param>
          <param name="baud_rate">${baud_rate}</param>
          <param name="control_mode">position</param>
        </xacro:unless>
      </hardware>

      <!-- Joint 1: Shoulder Pan -->
      <joint name="${prefix}shoulder_pan_joint">
        <param name="id">1</param>
        <param name="min_position_rad">-3.14159</param>
        <param name="max_position_rad">3.14159</param>
        <param name="min_position_raw">0</param>
        <param name="max_position_raw">4095</param>
        <command_interface name="position">
          <param name="min">-3.14159</param>
          <param name="max">3.14159</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="temperature"/>
      </joint>

      <!-- Joint 2: Shoulder Lift -->
      <joint name="${prefix}shoulder_lift_joint">
        <param name="id">2</param>
        <param name="min_position_rad">-1.57</param>
        <param name="max_position_rad">1.57</param>
        <param name="min_position_raw">512</param>
        <param name="max_position_raw">3584</param>
        <command_interface name="position">
          <param name="min">-1.57</param>
          <param name="max">1.57</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="temperature"/>
      </joint>

      <!-- Joint 3: Elbow -->
      <joint name="${prefix}elbow_joint">
        <param name="id">3</param>
        <param name="min_position_rad">-2.35</param>
        <param name="max_position_rad">2.35</param>
        <param name="min_position_raw">256</param>
        <param name="max_position_raw">3840</param>
        <command_interface name="position">
          <param name="min">-2.35</param>
          <param name="max">2.35</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="temperature"/>
      </joint>

      <!-- Joint 4: Wrist 1 -->
      <joint name="${prefix}wrist_1_joint">
        <param name="id">4</param>
        <param name="min_position_rad">-3.14159</param>
        <param name="max_position_rad">3.14159</param>
        <param name="min_position_raw">0</param>
        <param name="max_position_raw">4095</param>
        <command_interface name="position">
          <param name="min">-3.14159</param>
          <param name="max">3.14159</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="temperature"/>
      </joint>

      <!-- Joint 5: Wrist 2 -->
      <joint name="${prefix}wrist_2_joint">
        <param name="id">5</param>
        <param name="min_position_rad">-1.57</param>
        <param name="max_position_rad">1.57</param>
        <param name="min_position_raw">512</param>
        <param name="max_position_raw">3584</param>
        <command_interface name="position">
          <param name="min">-1.57</param>
          <param name="max">1.57</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="temperature"/>
      </joint>

      <!-- Joint 6: Wrist 3 -->
      <joint name="${prefix}wrist_3_joint">
        <param name="id">6</param>
        <param name="min_position_rad">-3.14159</param>
        <param name="max_position_rad">3.14159</param>
        <param name="min_position_raw">0</param>
        <param name="max_position_raw">4095</param>
        <command_interface name="position">
          <param name="min">-3.14159</param>
          <param name="max">3.14159</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="temperature"/>
      </joint>

    </ros2_control>

  </xacro:macro>

</robot>
```

**Note:** Replace the min/max position values with your actual calibrated values from SO-100 specifications.

### 3.5 Update Main URDF to Include ros2_control

**File:** `urdf/so100_arm.urdf.xacro` (modify your existing file)

Add this near the top:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="so100_arm">

  <!-- Include ros2_control configuration -->
  <xacro:include filename="$(find perseus_lite_arm_description)/urdf/so100_arm.ros2_control.xacro"/>

  <!-- Your existing arm definition here -->
  <!-- ... links, joints, meshes ... -->

  <!-- Instantiate ros2_control -->
  <xacro:so100_arm_ros2_control
    name="so100_arm_hardware"
    serial_port="/dev/ttyUSB0"
    use_mock_hardware="false"/>

</robot>
```

### 3.6 Create Launch File for Visualization

**File:** `launch/view_arm.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware (for visualization only)",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("perseus_lite_arm_description"), "urdf", "so100_arm.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint State Publisher GUI (for manual control in visualization)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=LaunchConfigurationEquals("use_mock_hardware", "true"),
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("perseus_lite_arm_description"), "rviz", "view_arm.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
```

### 3.7 Create RViz Configuration

**File:** `rviz/view_arm.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid

    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false

    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true

  Global Options:
    Fixed Frame: base_link

  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.5
```

### 3.8 Update CMakeLists.txt

**File:** `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(perseus_lite_arm_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 3.9 Update package.xml

**File:** `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perseus_lite_arm_description</name>
  <version>0.1.0</version>
  <description>URDF description for Perseus-lite SO-100 robotic arm</description>

  <maintainer email="your-email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>urdf</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>perseus_lite_hardware</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 3.10 Build and Test

```bash
cd /home/dingo/perseus-v2/software/ros_ws

# Build
colcon build --packages-select perseus_lite_arm_description

# Source
source install/setup.bash

# Test visualization
ros2 launch perseus_lite_arm_description view_arm.launch.py
```

**Expected Result:** RViz opens showing SO-100 arm. Use Joint State Publisher GUI to move joints.

**Checkpoint:** SO-100 URDF visualizes correctly in RViz with all 6 joints controllable.

---

## Phase 4: MoveIt! 2 Configuration

**Objective:** Generate MoveIt configuration package using Setup Assistant.

### 4.1 Launch MoveIt Setup Assistant

```bash
cd /home/dingo/perseus-v2/software/ros_ws

# Source workspace
source install/setup.bash

# Launch Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### 4.2 Setup Assistant Workflow

#### Step 1: Start

- Click "Create New MoveIt Configuration Package"
- Browse to: `/home/dingo/perseus-v2/software/ros_ws/install/perseus_lite_arm_description/share/perseus_lite_arm_description/urdf/so100_arm.urdf.xacro`
- Click "Load Files"

#### Step 2: Self-Collisions

- Click "Self-Collisions" tab
- Click "Generate Collision Matrix"
- Sampling density: 10000 (default is fine)
- Wait for computation to complete
- Review collision pairs, disable false positives if needed

#### Step 3: Virtual Joints

- Click "Virtual Joints" tab
- Add virtual joint:
  - Joint Name: `world_to_base`
  - Child Link: `base_link` (or your arm's base link)
  - Parent Frame Name: `world`
  - Joint Type: `fixed`

#### Step 4: Planning Groups

- Click "Planning Groups" tab
- Add Group:
  - Group Name: `arm`
  - Kinematic Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
  - Click "Add Joints"
  - Select all 6 arm joints:
    - `shoulder_pan_joint`
    - `shoulder_lift_joint`
    - `elbow_joint`
    - `wrist_1_joint`
    - `wrist_2_joint`
    - `wrist_3_joint`
  - Click "Save"

#### Step 5: Robot Poses

- Click "Robot Poses" tab
- Add poses for testing:
  - **Home:**
    - Name: `home`
    - Set all joints to 0.0
  - **Stowed:**
    - Name: `stowed`
    - Set joints to compact storage position
  - **Ready:**
    - Name: `ready`
    - Set joints to ready-to-work position

#### Step 6: End Effectors

- Click "End Effectors" tab
- Add end effector (if you have a gripper):
  - End Effector Name: `gripper`
  - End Effector Group: (create separate group if gripper has joints)
  - Parent Link: `wrist_3_link` (or your tool flange link)
  - Parent Group: `arm`

**If no gripper yet:** Skip this step for now, can add later.

#### Step 7: Passive Joints

- Click "Passive Joints" tab
- Leave empty (SO-100 has no passive joints)

#### Step 8: ROS 2 Controllers

- Click "ROS 2 Controllers" tab
- Auto-detect from URDF or add manually:
  - Controller Name: `arm_controller`
  - Controller Type: `joint_trajectory_controller/JointTrajectoryController`
  - Add all 6 arm joints

#### Step 9: Author Information

- Fill in your name and email

#### Step 10: Configuration Files

- Click "Configuration Files" tab
- Choose output location:
  - `/home/dingo/perseus-v2/software/ros_ws/src/perseus_lite_arm_moveit_config`
- Click "Generate Package"

### 4.3 Post-Generation Modifications

#### 4.3.1 Update Joint Limits

**File:** `config/joint_limits.yaml`

Add realistic velocity and acceleration limits based on ST3215 capabilities:

```yaml
joint_limits:
  shoulder_pan_joint:
    has_velocity_limits: true
    max_velocity: 2.0 # rad/s (adjust based on your arm specs)
    has_acceleration_limits: true
    max_acceleration: 3.0 # rad/s²
    has_jerk_limits: false

  shoulder_lift_joint:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 3.0
    has_jerk_limits: false

  elbow_joint:
    has_velocity_limits: true
    max_velocity: 2.5
    has_acceleration_limits: true
    max_acceleration: 4.0
    has_jerk_limits: false

  wrist_1_joint:
    has_velocity_limits: true
    max_velocity: 3.0
    has_acceleration_limits: true
    max_acceleration: 5.0
    has_jerk_limits: false

  wrist_2_joint:
    has_velocity_limits: true
    max_velocity: 3.0
    has_acceleration_limits: true
    max_acceleration: 5.0
    has_jerk_limits: false

  wrist_3_joint:
    has_velocity_limits: true
    max_velocity: 3.0
    has_acceleration_limits: true
    max_acceleration: 5.0
    has_jerk_limits: false
```

**Note:** ST3215 max speed is ±1000 RPM ≈ 104.7 rad/s. These conservative limits account for load and safety.

#### 4.3.2 Configure Kinematics Solver

**File:** `config/kinematics.yaml`

Option 1 - KDL (built-in, fast, but basic):

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
```

Option 2 - TRAC-IK (recommended for better performance):

```yaml
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  solve_type: Speed # or Distance or Manipulation1/2
```

**To use TRAC-IK:** Install it first:

```bash
sudo apt install ros-${ROS_DISTRO}-trac-ik-kinematics-plugin
```

#### 4.3.3 Update MoveIt Controllers Configuration

**File:** `config/moveit_controllers.yaml`

```yaml
# MoveIt uses this configuration for executing motion plans
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
```

#### 4.3.4 Optimize OMPL Planning

**File:** `config/ompl_planning.yaml`

Add SO-100-specific tuning:

```yaml
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/ResolveConstraintFrames
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints

start_state_max_bounds_error: 0.1

planner_configs:
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0 # 0.0 = auto-detect
  RRT:
    type: geometric::RRT
    range: 0.0
  LBKPIECE:
    type: geometric::LBKPIECE
    range: 0.0
    border_fraction: 0.9
    min_valid_path_fraction: 0.5

arm:
  default_planner_config: RRTConnect
  planner_configs:
    - RRTConnect
    - RRT
    - LBKPIECE
  projection_evaluator: joints(shoulder_pan_joint,shoulder_lift_joint)
  longest_valid_segment_fraction: 0.005
```

### 4.4 Build MoveIt Config Package

```bash
cd /home/dingo/perseus-v2/software/ros_ws

# Build
colcon build --packages-select perseus_lite_arm_moveit_config

# Source
source install/setup.bash
```

### 4.5 Test MoveIt Demo

```bash
# Launch MoveIt demo (with mock hardware)
ros2 launch perseus_lite_arm_moveit_config demo.launch.py
```

**Expected Result:**

- RViz opens with MoveIt Motion Planning plugin
- You can drag the interactive marker at the end effector
- Planning and execution works in simulation

**Checkpoint:** MoveIt plans and executes trajectories in simulation mode.

---

## Phase 5: Controller Configuration

**Objective:** Create controller configuration and launch files for hardware integration.

### 5.1 Create Controllers Package

```bash
cd /home/dingo/perseus-v2/software/ros_ws/src

ros2 pkg create perseus_lite_arm_controllers \
    --build-type ament_cmake \
    --dependencies controller_manager \
                 joint_trajectory_controller \
                 robot_state_publisher

cd perseus_lite_arm_controllers
mkdir -p config launch
```

### 5.2 Controller Configuration

**File:** `config/arm_controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50 # Hz - matches ST3215 communication rate

    # Controller list
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# Joint State Broadcaster - publishes joint states to /joint_states
joint_state_broadcaster:
  ros__parameters:
    # No additional parameters needed

# Arm Trajectory Controller - receives trajectories from MoveIt
arm_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

    # Command interfaces (what we send to hardware)
    command_interfaces:
      - position

    # State interfaces (what we receive from hardware)
    state_interfaces:
      - position
      - velocity

    # Publishing
    state_publish_rate: 50.0
    action_monitor_rate: 20.0

    # Tolerances
    allow_partial_joints_goal: false
    allow_integration_in_goal_trajectories: true

    constraints:
      stopped_velocity_tolerance: 0.02 # rad/s
      goal_time: 0.0 # No time constraint (0 = disabled)

      # Per-joint goal tolerances
      shoulder_pan_joint:
        trajectory: 0.05 # rad
        goal: 0.02 # rad
      shoulder_lift_joint:
        trajectory: 0.05
        goal: 0.02
      elbow_joint:
        trajectory: 0.05
        goal: 0.02
      wrist_1_joint:
        trajectory: 0.05
        goal: 0.02
      wrist_2_joint:
        trajectory: 0.05
        goal: 0.02
      wrist_3_joint:
        trajectory: 0.05
        goal: 0.02
```

### 5.3 Hardware Bringup Launch File

**File:** `launch/arm_bringup.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for ST3215 servos",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start with mock hardware (simulation)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="arm_controllers.yaml",
            description="YAML file with controller configuration",
        )
    )

    # Initialize arguments
    serial_port = LaunchConfiguration("serial_port")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controllers_file = LaunchConfiguration("controllers_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_lite_arm_description"),
                    "urdf",
                    "so100_arm.urdf.xacro",
                ]
            ),
            " ",
            "serial_port:=",
            serial_port,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("perseus_lite_arm_controllers"),
            "config",
            controllers_file,
        ]
    )

    # Controller Manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot State Publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Arm Controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # Delay arm controller spawner after joint state broadcaster
    delay_arm_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner_after_joint_state_broadcaster,
    ]

    return LaunchDescription(declared_arguments + nodes)
```

### 5.4 MoveIt Integration Launch File

**File:** `launch/arm_moveit.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for ST3215 servos",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start with mock hardware",
        )
    )

    # Initialize arguments
    serial_port = LaunchConfiguration("serial_port")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Include arm hardware bringup
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_lite_arm_controllers"),
                    "launch",
                    "arm_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "serial_port": serial_port,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    # Include MoveIt move_group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_lite_arm_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    # Include MoveIt RViz
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_lite_arm_moveit_config"),
                    "launch",
                    "moveit_rviz.launch.py",
                ]
            )
        ),
    )

    return LaunchDescription(
        declared_arguments + [arm_bringup, move_group, moveit_rviz]
    )
```

### 5.5 Update Package Files

**File:** `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(perseus_lite_arm_controllers)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

**File:** `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perseus_lite_arm_controllers</name>
  <version>0.1.0</version>
  <description>ROS 2 controllers for Perseus-lite SO-100 arm</description>

  <maintainer email="your-email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>perseus_lite_arm_description</exec_depend>
  <exec_depend>perseus_lite_arm_moveit_config</exec_depend>
  <exec_depend>perseus_lite_hardware</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 5.6 Build and Test

```bash
cd /home/dingo/perseus-v2/software/ros_ws

# Build
colcon build --packages-select perseus_lite_arm_controllers

# Source
source install/setup.bash

# Test hardware bringup (simulation mode first)
ros2 launch perseus_lite_arm_controllers arm_bringup.launch.py use_mock_hardware:=true
```

**Expected Output:**

```
[INFO] [controller_manager]: Loaded controller 'joint_state_broadcaster'
[INFO] [controller_manager]: Loaded controller 'arm_controller'
[INFO] [spawner]: Loaded joint_state_broadcaster
[INFO] [spawner]: Loaded arm_controller
```

**Verify controllers:**

```bash
# List controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# arm_controller[joint_trajectory_controller/JointTrajectoryController] active
```

**Test trajectory execution:**

```bash
# Send a simple trajectory
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
      points: [
        { positions: [0.5, 0.5, 0.5, 0.0, 0.0, 0.0], time_from_start: {sec: 2} }
      ]
    }
  }"
```

**Checkpoint:** Controllers load successfully and execute trajectories in simulation.

---

## Phase 6: Example Applications & Testing

**Objective:** Create Python example scripts demonstrating arm control and MoveIt usage.

### 6.1 Create Examples Package

```bash
cd /home/dingo/perseus-v2/software/ros_ws/src

ros2 pkg create perseus_lite_arm_examples \
    --build-type ament_python \
    --dependencies rclpy moveit_py std_msgs sensor_msgs

cd perseus_lite_arm_examples
mkdir -p config
```

### 6.2 Test Hardware Communication

**File:** `perseus_lite_arm_examples/test_hardware.py`

```python
#!/usr/bin/env python3
"""
Test ST3215 servo communication and state feedback.
Verifies hardware interface is working correctly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HardwareTestNode(Node):
    def __init__(self):
        super().__init__('hardware_test_node')

        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.get_logger().info('Hardware test node started')
        self.get_logger().info('Listening for joint states...')

        self.received_states = False

    def joint_states_callback(self, msg):
        if not self.received_states:
            self.get_logger().info('Successfully receiving joint states!')
            self.received_states = True

        # Print joint positions
        for name, pos in zip(msg.name, msg.position):
            self.get_logger().info(f'{name}: {pos:.3f} rad', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.3 Home Arm Script

**File:** `perseus_lite_arm_examples/home_arm.py`

```python
#!/usr/bin/env python3
"""
Move arm to home position using MoveIt.
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class HomeArmNode(Node):
    def __init__(self):
        super().__init__('home_arm_node')

        # Initialize MoveIt
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")

        self.get_logger().info('MoveIt initialized')

    def move_to_home(self):
        self.get_logger().info('Moving to home position...')

        # Set target to named state
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")

        # Plan
        plan_result = self.arm.plan()

        if plan_result:
            self.get_logger().info('Plan found, executing...')
            # Execute
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, blocking=True)
            self.get_logger().info('Motion complete!')
        else:
            self.get_logger().error('Planning failed!')


def main(args=None):
    rclpy.init(args=args)
    node = HomeArmNode()

    # Move to home
    node.move_to_home()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.4 Plan to Pose Script

**File:** `perseus_lite_arm_examples/plan_to_pose.py`

```python
#!/usr/bin/env python3
"""
Plan and execute motion to a target pose.
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


class PlanToPoseNode(Node):
    def __init__(self):
        super().__init__('plan_to_pose_node')

        # Initialize MoveIt
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")

        self.get_logger().info('MoveIt initialized')

    def plan_to_pose(self, x, y, z, roll, pitch, yaw):
        self.get_logger().info(f'Planning to pose: x={x}, y={y}, z={z}')

        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        # Convert RPY to quaternion
        from scipy.spatial.transform import Rotation
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]

        # Set goal
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="wrist_3_link")

        # Plan
        plan_result = self.arm.plan()

        if plan_result:
            self.get_logger().info('Plan found, executing...')
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, blocking=True)
            self.get_logger().info('Motion complete!')
            return True
        else:
            self.get_logger().error('Planning failed!')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = PlanToPoseNode()

    # Example: Move to pose 30cm forward, 10cm right, 20cm up
    node.plan_to_pose(x=0.3, y=-0.1, z=0.2, roll=0.0, pitch=1.57, yaw=0.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.5 Cartesian Path Script

**File:** `perseus_lite_arm_examples/cartesian_path.py`

```python
#!/usr/bin/env python3
"""
Execute a Cartesian path (straight line motion).
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose
import numpy as np


class CartesianPathNode(Node):
    def __init__(self):
        super().__init__('cartesian_path_node')

        # Initialize MoveIt
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")

        self.get_logger().info('MoveIt initialized')

    def execute_cartesian_path(self, waypoints):
        """
        Execute Cartesian path through waypoints.

        Args:
            waypoints: List of Pose objects
        """
        self.get_logger().info(f'Planning Cartesian path with {len(waypoints)} waypoints')

        # Compute Cartesian path
        fraction = self.arm.compute_cartesian_path(
            waypoints=waypoints,
            max_step=0.01,  # 1cm resolution
            jump_threshold=0.0,  # Disable jump detection
        )

        if fraction > 0.95:  # At least 95% of path achieved
            self.get_logger().info(f'Cartesian path computed ({fraction*100:.1f}% achieved)')

            # Execute
            plan_result = self.arm.plan()
            if plan_result:
                robot_trajectory = plan_result.trajectory
                self.moveit.execute(robot_trajectory, blocking=True)
                self.get_logger().info('Cartesian motion complete!')
                return True
        else:
            self.get_logger().error(f'Cartesian path planning failed (only {fraction*100:.1f}% achieved)')

        return False


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPathNode()

    # Create waypoints for a square motion
    waypoints = []

    # Get current pose
    current_state = node.moveit.get_robot_model().get_current_state()
    start_pose = current_state.get_pose("wrist_3_link")

    # Move right 10cm
    pose1 = Pose()
    pose1.position.x = start_pose.position.x
    pose1.position.y = start_pose.position.y - 0.1
    pose1.position.z = start_pose.position.z
    pose1.orientation = start_pose.orientation
    waypoints.append(pose1)

    # Move forward 10cm
    pose2 = Pose()
    pose2.position.x = start_pose.position.x + 0.1
    pose2.position.y = pose1.position.y
    pose2.position.z = start_pose.position.z
    pose2.orientation = start_pose.orientation
    waypoints.append(pose2)

    # Execute
    node.execute_cartesian_path(waypoints)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.6 Update setup.py

**File:** `setup.py`

```python
from setuptools import setup
from glob import glob
import os

package_name = 'perseus_lite_arm_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Example applications for Perseus-lite SO-100 arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_hardware = perseus_lite_arm_examples.test_hardware:main',
            'home_arm = perseus_lite_arm_examples.home_arm:main',
            'plan_to_pose = perseus_lite_arm_examples.plan_to_pose:main',
            'cartesian_path = perseus_lite_arm_examples.cartesian_path:main',
        ],
    },
)
```

### 6.7 Build and Test Examples

```bash
cd /home/dingo/perseus-v2/software/ros_ws

# Build
colcon build --packages-select perseus_lite_arm_examples

# Source
source install/setup.bash

# Test (in simulation first)
# Terminal 1: Launch MoveIt
ros2 launch perseus_lite_arm_controllers arm_moveit.launch.py use_mock_hardware:=true

# Terminal 2: Run examples
ros2 run perseus_lite_arm_examples test_hardware
ros2 run perseus_lite_arm_examples home_arm
```

**Checkpoint:** Example scripts successfully control the arm in simulation.

---

## Phase 7: Integration Testing & Tuning

**Objective:** Test with physical hardware and tune parameters for optimal performance.

### 7.1 Hardware Setup

#### 7.1.1 Serial Port Configuration

Create udev rule for persistent device naming:

**File:** `/etc/udev/rules.d/99-st3215-arm.rules` (requires sudo)

```bash
# SO-100 Arm ST3215 Serial Interface
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="ttyUSB_ARM"
```

**Find your vendor/product ID:**

```bash
# Plug in USB-serial adapter
dmesg | tail

# Find device
ls -l /dev/ttyUSB* /dev/ttyACM*

# Get vendor/product ID
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
```

**Reload udev rules:**

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 7.1.2 Serial Port Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in for changes to take effect
```

### 7.2 Hardware Testing Procedure

#### Test 1: Serial Communication

```bash
# Test serial port
sudo chmod 666 /dev/ttyUSB0  # Temporary for testing
echo "Testing" > /dev/ttyUSB0

# Launch hardware interface
ros2 launch perseus_lite_arm_controllers arm_bringup.launch.py \
  serial_port:=/dev/ttyUSB0 \
  use_mock_hardware:=false
```

**Expected:** No errors, joint states published

#### Test 2: Joint State Feedback

```bash
# Monitor joint states
ros2 topic echo /joint_states

# Expected output: Position, velocity, effort for all 6 joints updating at ~50Hz
```

#### Test 3: Manual Joint Control

```bash
# Send simple trajectory
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
      points: [
        { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3} }
      ]
    }
  }"
```

**Expected:** Arm moves smoothly to home position

#### Test 4: MoveIt Planning

```bash
# Launch full MoveIt stack
ros2 launch perseus_lite_arm_controllers arm_moveit.launch.py \
  serial_port:=/dev/ttyUSB0 \
  use_mock_hardware:=false
```

**In RViz:**

- Drag interactive marker
- Click "Plan"
- Click "Execute"
- Arm should follow planned path

### 7.3 Parameter Tuning

#### 7.3.1 ST3215 Servo Tuning

Test different acceleration values in hardware interface:

```bash
# Access servo parameter server (if exposed)
ros2 param set /controller_manager/so100_arm_hardware acceleration 50

# Values to try: 10 (slow/smooth), 30 (default), 100 (fast/jerky)
```

#### 7.3.2 Controller Tolerance Tuning

Edit `config/arm_controllers.yaml`:

```yaml
arm_controller:
  ros__parameters:
    constraints:
      # If arm overshoots, tighten tolerances
      stopped_velocity_tolerance: 0.01 # Decrease from 0.02

      shoulder_pan_joint:
        goal: 0.01 # Decrease from 0.02 for tighter control
```

Rebuild and retest:

```bash
colcon build --packages-select perseus_lite_arm_controllers
```

#### 7.3.3 MoveIt Planning Time

Edit `config/ompl_planning.yaml`:

```yaml
# If planning too slow
default_planner_config: RRTConnect # Faster than RRT
planner_configs:
  RRTConnect:
    range: 0.1 # Increase for faster planning (less precision)
```

### 7.4 Safety Testing

#### 7.4.1 Joint Limits

**Verify limits are enforced:**

```bash
# Try to command beyond limit
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [shoulder_pan_joint],
      points: [
        { positions: [10.0], time_from_start: {sec: 3} }
      ]
    }
  }"
```

**Expected:** Command rejected or clamped to limit

#### 7.4.2 Collision Avoidance

**Test self-collision:**

- In RViz, plan to pose that would cause collision
- MoveIt should reject plan

**Test scene collisions:**

- Add obstacle to planning scene
- Plan through obstacle
- Should find path around it

#### 7.4.3 Emergency Stop

**Implement e-stop:**

- Kill ros2_control_node kills all motion
- Test: `pkill -9 ros2_control_node`
- Servos should stop immediately (torque off)

### 7.5 Performance Benchmarking

#### 7.5.1 Planning Speed

```python
import time
# In plan_to_pose.py, add timing:
start = time.time()
plan_result = self.arm.plan()
end = time.time()
print(f"Planning time: {end - start:.3f}s")
```

**Target:** < 1 second for simple motions

#### 7.5.2 Execution Accuracy

```python
# After execution, check final position error
final_state = self.moveit.get_robot_model().get_current_state()
error = calculate_pose_error(target_pose, final_state.get_pose("wrist_3_link"))
print(f"Position error: {error:.3f}m")
```

**Target:** < 5mm position error

#### 7.5.3 Trajectory Smoothness

```bash
# Record joint states during execution
ros2 bag record /joint_states

# Plot in rqt_bag or plotjuggler
# Check for smooth velocity profiles (no discontinuities)
```

### 7.6 Common Issues & Solutions

| Issue              | Symptom                                          | Solution                                             |
| ------------------ | ------------------------------------------------ | ---------------------------------------------------- |
| Servo timeout      | Hardware interface error "timeout reading servo" | Check baud rate, cable quality, servo IDs            |
| Jerky motion       | Arm moves in steps not smoothly                  | Decrease acceleration, increase trajectory points    |
| Position overshoot | Arm oscillates around goal                       | Tighten controller tolerances, tune servo PID        |
| Planning failures  | MoveIt cannot find path                          | Increase planning time, change planner, check limits |
| Slow planning      | Plans take >5 seconds                            | Use RRTConnect, increase range parameter             |
| Collisions ignored | Arm hits itself                                  | Regenerate collision matrix, check URDF geometry     |

### 7.7 Calibration Refinement

Run calibration script to fine-tune joint limits:

```bash
ros2 run perseus_lite_arm_examples calibrate_joints

# Follow prompts to move each joint to limits
# Script generates updated joint_calibration.yaml
# Copy to perseus_lite_arm_description/config/
```

### 7.8 Final Integration Test

**Complete workflow test:**

1. Power on system
2. Launch MoveIt: `ros2 launch perseus_lite_arm_controllers arm_moveit.launch.py`
3. Run home script: `ros2 run perseus_lite_arm_examples home_arm`
4. Plan to multiple poses using RViz
5. Execute Cartesian path
6. Check all safety limits enforced
7. Verify smooth, accurate motion

**Success criteria:**

- [ ] All 6 joints move independently
- [ ] Position accuracy < 5mm
- [ ] Planning time < 2s for typical motions
- [ ] No collisions (self or environment)
- [ ] Joint limits enforced
- [ ] Smooth trajectories (no jerking)
- [ ] Temperature < 70°C during operation
- [ ] Can run continuously for 30 minutes

**Checkpoint:** Physical arm executes MoveIt-planned trajectories accurately and safely.

---

## Technical Reference

### ST3215 Register Map (Key Registers)

| Register            | Address | Type | Range          | Description              |
| ------------------- | ------- | ---- | -------------- | ------------------------ |
| ID                  | 0x05    | R/W  | 0-253          | Servo ID                 |
| BAUD_RATE           | 0x06    | R/W  | 0-7            | Baud rate setting        |
| TORQUE_ENABLE       | 0x28    | R/W  | 0-1            | Enable/disable torque    |
| ACCELERATION        | 0x29    | R/W  | 0-255          | Acceleration value       |
| GOAL_POSITION       | 0x2A    | R/W  | 0-4095         | Target position (12-bit) |
| RUNNING_TIME        | 0x2C    | R/W  | 0-65535        | Movement duration (ms)   |
| GOAL_SPEED          | 0x2E    | R/W  | -1000 to +1000 | Target speed (RPM)       |
| TORQUE_LIMIT        | 0x30    | R/W  | 0-1000         | Max torque limit         |
| PRESENT_POSITION    | 0x38    | R    | 0-4095         | Current position         |
| PRESENT_SPEED       | 0x3A    | R    | -1000 to +1000 | Current speed (RPM)      |
| PRESENT_LOAD        | 0x3C    | R    | -1000 to +1000 | Current load/torque      |
| PRESENT_VOLTAGE     | 0x3E    | R    | 0-255          | Voltage (0.1V units)     |
| PRESENT_TEMPERATURE | 0x3F    | R    | 0-255          | Temperature (°C)         |

### ROS 2 Topics

| Topic                                     | Type                               | Description                         |
| ----------------------------------------- | ---------------------------------- | ----------------------------------- |
| `/joint_states`                           | sensor_msgs/JointState             | Current joint positions, velocities |
| `/robot_description`                      | std_msgs/String                    | URDF robot model                    |
| `/arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | Trajectory action server            |
| `/move_group/display_planned_path`        | moveit_msgs/DisplayTrajectory      | Planned path visualization          |
| `/planning_scene`                         | moveit_msgs/PlanningScene          | Collision environment               |

### ROS 2 Parameters

**Controller Manager:**

- `update_rate`: Control loop frequency (Hz)

**Joint Trajectory Controller:**

- `joints`: List of joint names
- `command_interfaces`: [position]
- `state_interfaces`: [position, velocity]
- `constraints.goal_time`: Goal time tolerance (s)
- `constraints.<joint>.goal`: Goal position tolerance (rad)

**ST3215 Hardware Interface:**

- `serial_port`: Serial device path
- `baud_rate`: Communication baud rate
- `control_mode`: "position" or "velocity"
- `calibration_file`: Path to calibration YAML

### Useful Commands

```bash
# List controllers
ros2 control list_controllers

# Get controller info
ros2 control list_hardware_interfaces

# Monitor joint states
ros2 topic echo /joint_states

# Send test trajectory
ros2 action send_goal /arm_controller/follow_joint_trajectory <trajectory>

# Check MoveIt planning pipeline
ros2 topic list | grep move_group

# View TF tree
ros2 run tf2_tools view_frames

# Plot joint data
ros2 run plotjuggler plotjuggler
```

### Conversion Factors

| From                 | To                   | Factor                              |
| -------------------- | -------------------- | ----------------------------------- |
| Servo position (raw) | Radians              | `(raw / 4095) * 2π - π`             |
| Radians              | Servo position (raw) | `((rad + π) / (2π)) * 4095`         |
| RPM                  | Rad/s                | `RPM * (2π / 60) = RPM * 0.1047`    |
| Rad/s                | RPM                  | `rad_s * (60 / 2π) = rad_s * 9.549` |

---

## Success Criteria

### Phase 2: Hardware Interface

- [ ] ST3215SystemHardware exports position command interface
- [ ] Calibration data loads from YAML or parameters
- [ ] Position mapping (radians ↔ servo raw) works correctly
- [ ] Builds without errors

### Phase 3: Robot Description

- [ ] SO-100 URDF loads in RViz
- [ ] All 6 joints visible and movable
- [ ] Meshes display correctly
- [ ] ros2_control tag properly configured
- [ ] Joint limits defined

### Phase 4: MoveIt Configuration

- [ ] MoveIt Setup Assistant generates package
- [ ] Planning group "arm" contains 6 joints
- [ ] IK solver configured
- [ ] Demo mode works (mock hardware)
- [ ] Can plan and execute in simulation

### Phase 5: Controllers

- [ ] JointStateBroadcaster publishes to /joint_states
- [ ] JointTrajectoryController accepts trajectories
- [ ] Controllers spawn without errors
- [ ] Hardware bringup launch file works
- [ ] MoveIt launch file integrates all components

### Phase 6: Examples

- [ ] test_hardware.py receives joint states
- [ ] home_arm.py moves to home position
- [ ] plan_to_pose.py plans and executes
- [ ] All examples run in simulation

### Phase 7: Physical Integration

- [ ] Serial communication working (no timeouts)
- [ ] Joint states update at 50Hz
- [ ] Physical arm executes trajectories
- [ ] Position accuracy < 5mm
- [ ] MoveIt planning works with real hardware
- [ ] Safety limits enforced
- [ ] No self-collisions
- [ ] Temperature stays < 70°C
- [ ] Can run for 30+ minutes continuously

---

## Appendix: Troubleshooting

### Serial Communication Issues

**Problem:** `Cannot open serial port /dev/ttyUSB0`

```bash
# Check permissions
ls -l /dev/ttyUSB0

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in

# Temporarily test with sudo
sudo chmod 666 /dev/ttyUSB0
```

**Problem:** `Timeout reading servo`

```bash
# Check baud rate matches servo configuration
# Default should be 115200

# Test with lower baud rate
serial_port:=/dev/ttyUSB0 baud_rate:=9600

# Check servo IDs are correct (1-6)
# Verify servo is powered and connected
```

### Controller Issues

**Problem:** `Controller failed to activate`

```bash
# Check if hardware interface loaded
ros2 control list_hardware_interfaces

# Check controller configuration
ros2 param list /controller_manager

# Restart controller manager
ros2 lifecycle set /controller_manager configure
ros2 lifecycle set /controller_manager activate
```

**Problem:** `Trajectory execution fails`

```bash
# Check joint limits
ros2 param get /arm_controller joints

# Verify state interfaces available
ros2 control list_hardware_interfaces | grep state

# Check for controller warnings
ros2 topic echo /arm_controller/controller_state
```

### MoveIt Issues

**Problem:** `IK solver fails`

```bash
# Try different solver
# Edit config/kinematics.yaml, change to TRAC-IK

# Increase timeout
kinematics_solver_timeout: 0.1  # from 0.05

# Check if target pose is reachable
# Use RViz to verify workspace
```

**Problem:** `Planning takes too long`

```bash
# Use faster planner
# Edit config/ompl_planning.yaml
default_planner_config: RRTConnect

# Reduce planning time
planner_configs:
  RRTConnect:
    range: 0.2  # Increase for speed
```

**Problem:** `Collisions detected when there are none`

```bash
# Regenerate collision matrix in Setup Assistant
# Or manually disable collision pair in SRDF

# Check collision geometry in URDF
# Visualize collision meshes in RViz
```

### Hardware Interface Issues

**Problem:** `Joint positions don't match commanded values`

```bash
# Check calibration values
ros2 param get /controller_manager/so100_arm_hardware min_position_rad
ros2 param get /controller_manager/so100_arm_hardware max_position_rad

# Verify servo position reads correctly
ros2 topic echo /joint_states

# Check servo is in position control mode (not velocity mode)
```

---

## Next Steps After Integration

Once basic integration is complete, consider:

1. **Gripper Integration**

   - Add gripper URDF
   - Create gripper controller
   - Add to MoveIt as end effector

2. **Force/Torque Control**

   - Use PRESENT_LOAD register for force sensing
   - Implement compliant control modes
   - Add force limits

3. **Vision Integration**

   - Add camera to URDF
   - Configure MoveIt scene perception
   - Implement visual servoing

4. **Advanced Planning**

   - Custom constraints (orientation, path)
   - Multi-goal planning
   - Optimal path selection

5. **Teleoperation**

   - Joystick control
   - Leader-follower (already in arm-teleop-direct)
   - VR control

6. **Pick and Place**
   - Grasp planning
   - Object manipulation
   - Task-level programming

---

**Document Version:** 1.0
**Last Updated:** 2025-11-21
**Status:** Ready for execution
