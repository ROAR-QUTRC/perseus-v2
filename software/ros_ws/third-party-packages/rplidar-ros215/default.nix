{
  lib,
  buildRosPackage,
  fetchFromGitHub,
  ament-cmake,
  rclcpp,
  sensor-msgs,
  std-srvs,
}:

buildRosPackage {
  pname = "rplidar_ros";
  version = "2.1.5";

  src = fetchFromGitHub {
    owner = "Slamtec";
    repo = "rplidar_ros";
    rev = "db43a04ee5c2fd8ef826bbaa19be4bab7b167c4d"; # Commit for 2.1.5 - verify this
    sha256 = "sha256-0000000000000000000000000000000000000000000="; # Replace with real hash after first build attempt
  };

  buildType = "ament_cmake";
  buildInputs = [
    ament-cmake
    rclcpp
    sensor-msgs
    std-srvs
  ];

  meta = with lib; {
    description = "ROS2 node for Slamtec RPLidar C1";
    license = licenses.bsd3;
  };
}
