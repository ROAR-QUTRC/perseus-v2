# `sophus` is the lightweight top-level (nixpkgs) Sophus, passed in explicitly so
# vikit links against it rather than the ROS-scoped `ros-jazzy-sophus`, which
# builds Ceres/SuiteSparse from source and isn't in the binary cache.
final: prev:
let
  individualPackages = individualFinal: individualPrev: {
    livox-ros-driver2 = individualFinal.callPackage ./livox-ros-driver2 { };
    fast-lio = individualFinal.callPackage ./fast-lio { };
    ndt-omp-ros2 = individualFinal.callPackage ./ndt-omp-ros2 { };
    vikit-common = individualFinal.callPackage ./vikit-common { inherit (final) sophus; };
    vikit-ros = individualFinal.callPackage ./vikit-ros { inherit (final) sophus; };
    fast-livo2 = individualFinal.callPackage ./fast-livo2 { inherit (final) sophus; };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
  (import ./lidarslam-ros2/overlay.nix)
] final prev
