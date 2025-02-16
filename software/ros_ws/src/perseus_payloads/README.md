# About this Package

This package is intended to contain payload-specific code for Perseus. `*_driver` packages are intended to provide an interface between real hardware and ROS topics/services/actions - however, since they should make use of the most applicable ROS message types for their specific hardware, they may not be immediately useful to an end user.

This is where `*_controller` nodes come in.
They are intended to provide an interface between your _control_ messages (eg, `TwistStamped`) and the _driver nodes_ (which might take something like an `Actuators` message).
This isolates responsibilities - the driver nodes can focus on providing a transparent interface to hardware, and the controller nodes can focus on taking useful _input_ methods and converting that to hardware control.
