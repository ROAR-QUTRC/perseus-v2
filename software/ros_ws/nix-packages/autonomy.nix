# Automatically generated by: ros2nix --output-dir=ros_ws/nix-packages --output-as-nix-pkg-name --no-default --distro jazzy
{
  lib,
  buildRosPackage,
  ament-cmake,
  ament-lint-auto,
  ament-lint-common,
  rclcpp,
  ros2launch,
  rviz2,
  slam-toolbox,
  xacro,
}:
buildRosPackage rec {
  pname = "ros-jazzy-autonomy";
  version = "0.0.1";

  src = ./../src/autonomy;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  checkInputs = [
    ament-lint-auto
    ament-lint-common
  ];
  propagatedBuildInputs = [
    rclcpp
    ros2launch
    rviz2
    slam-toolbox
    xacro
  ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = "ROAR Autonomy software control stack";
    license = with lib.licenses; [ mit ];
  };
}
