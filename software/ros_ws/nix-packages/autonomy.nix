# Automatically generated by: ros2nix --output-dir=ros_ws/nix-packages --output-as-nix-pkg-name --no-default --distro jazzy
{
  lib,
  buildRosPackage,
  ament-cmake,
  ament-lint-auto,
  ament-lint-common,
  nav2-bringup,
  rclcpp,
  slam-toolbox,
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
    nav2-bringup
    rclcpp
    slam-toolbox
  ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = "ROAR Autonomy software control stack";
    license = with lib.licenses; [ mit ];
  };
}
