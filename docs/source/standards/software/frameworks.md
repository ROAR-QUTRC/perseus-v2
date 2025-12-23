---
tocdepth: 4
---

# Framework Specific Standards

Most frameworks will have their own documentation and development frameworks which should be read in conjunction with this document.
This page fills any gaps in each frameworks own documentation and also includes any other amendments necessary for our use case.

## ROS Standards

### Packages

Packages should contain a set of related nodes/launch files with similar responsibilities. For example, an `input_devices` package could contain nodes related to taking input from various devices and publishing that to the correct topics. Package names should not be prepended with anything (eg `rover_`) without a very good reason (such as a "meta-package" containing launch files).

### Nodes

Nodes should be named as _nouns_ according to what they _are_. Node names **should not** be appended with `_node`.

#### Acceptable

```
input_devices generic_controller
input_devices hotas_controller
input_devices joystick_controller
input_controller input_controller

autonomy costmap
autonomy lidar
autonomy mapper
```

### Launch Files

Launch files should be clearly named and handle errors appropriately. Config inputs should be clearly documented.

### Data transfer

#### Naming

The following applies to topics, services, and actions, but will name topics specifically for brevity.

Topics should be named hierarchically, typically with the first level of the name being the name of the package the node is in. For example, a node under `input_devices` might publish to `/input_devices/generic_controller`, or a node under autonomy might publish to `/autonomy/point_cloud`.

#### General

If there's an appropriate pre-existing ROS message type, use it. This simplifies development and also allows for greater interoperability with other pre-existing ROS software.
