# packages/livox-ros-driver2/default.nix
{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  pkg-config,
  ros2,
  livox-sdk2,
}:

stdenv.mkDerivation rec {
  pname = "livox-ros-driver2";
  version = "1.2.4";

  src = fetchFromGitHub {
    owner = "Livox-SDK";
    repo = "livox_ros_driver2";
    rev = version;
    hash = "sha256-XXXXX"; # You'll need to replace this with actual hash
  };

  nativeBuildInputs = [
    cmake
    pkg-config
  ];

  buildInputs = [
    ros2.humble.rclcpp
    ros2.humble.sensor-msgs
    ros2.humble.std-msgs
    ros2.humble.tf2
    ros2.humble.tf2-ros
    livox-sdk2
  ];

  cmakeFlags = [
    "-DROS_VERSION=2"
    "-DROS_DISTRO=humble"
  ];

  meta = with lib; {
    description = "Livox ROS Driver 2.0 for Livox LiDAR";
    homepage = "https://github.com/Livox-SDK/livox_ros_driver2";
    license = licenses.mit;
    maintainers = with maintainers; [ ];
    platforms = platforms.linux;
  };
}
