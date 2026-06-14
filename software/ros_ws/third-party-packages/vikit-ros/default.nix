# Vikit (Vision-Kit for Robotics) - ROS2 integration package.
# Source is the rpg_vikit monorepo; this package is the vikit_ros subdirectory.
{
  lib,
  buildRosPackage,
  fetchFromGitHub,
  ament-cmake,
  rosidl-default-generators,
  rosidl-default-runtime,
  rclcpp,
  tf2,
  tf2-ros,
  tf2-geometry-msgs,
  visualization-msgs,
  vikit-common,
  # non-ROS system dependencies (find_package'd directly in CMakeLists)
  eigen,
  opencv,
  sophus,
}:
let
  src = fetchFromGitHub {
    owner = "Robotic-Developer-Road";
    repo = "rpg_vikit";
    rev = "4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50";
    sha256 = "03sgaghr14fxm0yh2z8837j9722z6nmpp09srifkn12kilbc7hi6";
  };
in
buildRosPackage {
  pname = "ros-jazzy-vikit-ros";
  version = "0.0.0";

  inherit src;
  # rpg_vikit is a monorepo; build only the vikit_ros package
  sourceRoot = "${src.name}/vikit_ros";

  buildType = "ament_cmake";
  buildInputs = [
    ament-cmake
    rosidl-default-generators
  ];
  propagatedBuildInputs = [
    rclcpp
    tf2
    tf2-ros
    tf2-geometry-msgs
    visualization-msgs
    vikit-common
    eigen
    opencv
    sophus
    rosidl-default-runtime
  ];
  nativeBuildInputs = [
    ament-cmake
    rosidl-default-generators
  ];

  meta = {
    description = "Vikit ROS2 integration: camera parameter handling and ROS utilities";
    license = with lib.licenses; [ gpl3Plus ];
  };
}
