# Web-UI

The Perseus-UI is a web UI that is almost entirely independent of the rover core software. The UI is a [SvelteKit](https://svelte.dev/) + [Vite](https://vite.dev/) app with an injected Node.js server. There are two main components to Web-UI:

- Widgets -> Typically a single svelte component that provides an interface for a specific functionality on the rover.
- Layouts -> A group of widgets that are regularly used together.

The dependency vite is a very powerful development tool that is integrated with many popular web frameworks that allows us to do two very useful things:

1. Run a development server that hot reloads sections of HTML extremely fast while you develop new code
2. Build SvelteKit code into highly optimised "chunks" of static assets

## ROS2 Interface

Since web apps cannot directly access ROS2 topics and services a ROS2 node is required to act as a translation layer. The translation layer used for the Web-UI is the [rosbridge_suite](https://wiki.ros.org/rosbridge_suite) which translates all topics and service calls to JSON as per their [rosbrige protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/ROSBRIDGE_PROTOCOL.md), then publishes the JSON messages to a websocket. This is paired with the JavaScript client library [roslibjs](https://robotwebtools.github.io/roslibjs/index.html) which provides an easy to use abstraction from raw websockets and implements the rosbridge protocol.

Additionally to the roslibjs library the Web-UI has it's own interface implemented at the widget manager level to handle connections with roslibjs automatically and lock widgets dependant on ROS when connection is lost. This is further documented in the development section of these docs: <project:/development/software/web-ui.md#ros2-interface>.

## CAN Bus Interface

Presently due to limitations of the hi_can library there is no direct translation layer between CAN bus and the Web-UI. The current work around for this limitation is to create a ROS2 node that handles translating CAN bus messages to a ROS2 topic that is then accessed in the Web-UI via the ROS2 interface.

## Cameras

To gain the low level access that is required to operate cameras affectively a wrapper Node.js server has been created for GStreamer. This can be found in `software/web-ui/pi-server`. This secondary server primarily facilitates WebRTC connections as well as handle all the GStreamer instances and communicates to the Web-UI client to start, stop and resize camera feeds on demand.
