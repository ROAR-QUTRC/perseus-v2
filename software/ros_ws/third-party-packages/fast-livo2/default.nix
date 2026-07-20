# FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry (ROS2 Jazzy).
# ROAR-QUTRC fork of hku-mars/FAST-LIVO2, tracked on the `humble` branch.
#
# `sophus` is passed in explicitly (the lightweight top-level nixpkgs Sophus)
# rather than resolved from the ROS scope, which would pull ros-jazzy-sophus and
# its heavy Ceres/SuiteSparse build. Matches how vikit-common/vikit-ros are wired.
{
  lib,
  buildRosPackage,
  fetchFromGitHub,
  ament-cmake,
  rclcpp,
  rclpy,
  geometry-msgs,
  nav-msgs,
  sensor-msgs,
  std-msgs,
  visualization-msgs,
  tf2-ros,
  pcl-ros,
  pcl-conversions,
  cv-bridge,
  image-transport,
  livox-ros-driver2,
  vikit-common,
  vikit-ros,
  # non-ROS system dependencies
  eigen,
  pcl,
  opencv,
  sophus,
  fmt,
  boost,
}:
buildRosPackage {
  pname = "ros-jazzy-fast-livo2";
  version = "0.0.0";

  src = fetchFromGitHub {
    owner = "ROAR-QUTRC";
    repo = "FAST-LIVO2";
    rev = "837b7bbc1431cb04cf936528e52c83c835efba8e"; # humble branch HEAD
    sha256 = "100z3hahxq0bj17j5db7gvgp9cfkm40m47indwavamp5g3darahf";
  };

  # Upstream hardcodes colcon-workspace install paths for the vikit libraries
  # (../../install/vikit_*/lib/*.so), which don't exist under Nix. Point them at
  # the actual store paths of our vikit packages instead.
  postPatch = ''
    substituteInPlace CMakeLists.txt \
      --replace-fail "\''${CMAKE_SOURCE_DIR}/../../install/vikit_common/lib/libvikit_common.so" "${vikit-common}/lib/libvikit_common.so" \
      --replace-fail "\''${CMAKE_SOURCE_DIR}/../../install/vikit_ros/lib/libvikit_ros.so" "${vikit-ros}/lib/libvikit_ros.so"
  '';

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    rclcpp
    rclpy
    geometry-msgs
    nav-msgs
    sensor-msgs
    std-msgs
    visualization-msgs
    tf2-ros
    pcl-ros
    pcl-conversions
    cv-bridge
    image-transport
    livox-ros-driver2
    vikit-common
    vikit-ros
    eigen
    pcl
    opencv
    sophus
    fmt
    boost
  ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = "FAST-LIVO2: fast, direct LiDAR-inertial-visual odometry (ROAR-QUTRC ROS2 fork)";
    license = with lib.licenses; [ gpl2Only ];
  };
}
