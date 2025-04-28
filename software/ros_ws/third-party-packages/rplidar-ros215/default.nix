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
  version = "ros2";

  src = fetchFromGitHub {
    owner = "Slamtec";
    repo = "rplidar_ros";
    rev = "ros2";
    sha256 = "sha256-oNoDa+IqtQPe8bpfMjHFj2yx7jFUhfbIqaPRQCU/zMQ=";
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
