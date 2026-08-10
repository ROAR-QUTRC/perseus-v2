# `sophus`/`pkg-config`/`mpi` are the lightweight top-level (nixpkgs) packages, passed in
# explicitly so vikit links against nixpkgs' Sophus rather than the ROS-scoped
# `ros-jazzy-sophus` (which builds Ceres/SuiteSparse from source and isn't in the binary
# cache), and so mesh_msgs_conversions can satisfy its own unused find_package(MPI)/
# find_package(PkgConfig) calls.
#
# They have to arrive as parameters. `final` below is the ROS package set, not nixpkgs —
# this overlay is composed inside rosPkgsOverlay in ../overlay.nix — so `inherit (final)
# sophus` silently selects ros-jazzy-sophus and CI dies compiling its test suite.
{
  sophus,
  pkg-config,
  mpi,
}:
final: prev:
let
  individualPackages = individualFinal: individualPrev: {
    livox-ros-driver2 = individualFinal.callPackage ./livox-ros-driver2 { };
    fast-lio = individualFinal.callPackage ./fast-lio { };
    ndt-omp-ros2 = individualFinal.callPackage ./ndt-omp-ros2 { };
    vikit-common = individualFinal.callPackage ./vikit-common { inherit sophus; };
    vikit-ros = individualFinal.callPackage ./vikit-ros { inherit sophus; };
    fast-livo2 = individualFinal.callPackage ./fast-livo2 { inherit sophus; };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
  (import ./lidarslam-ros2/overlay.nix)
  (import ./mesh-tools/overlay.nix { inherit pkg-config mpi; })
  (import ./mesh-navigation/overlay.nix)
] final prev
