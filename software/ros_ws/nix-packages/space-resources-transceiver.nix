{
  lib,
  buildRosPackage,
  ament-copyright,
  ament-flake8,
  ament-pep257,
  perseus-interfaces,
  python3Packages,
  rclpy,
  std-msgs,
}:
buildRosPackage rec {
  pname = "ros-jazzy-space-resources-transceiver";
  version = "0.1.0";

  src = ./../src/space_resources_transceiver;

  buildType = "ament_python";
  checkInputs = [
    ament-copyright
    ament-flake8
    ament-pep257
    python3Packages.pytest
  ];
  propagatedBuildInputs = [
    perseus-interfaces
    python3Packages.smbus2
    rclpy
    std-msgs
  ];

  meta = {
    description = "915MHz transceiver bridge for space resources end effector";
    license = with lib.licenses; [ mit ];
  };
}
