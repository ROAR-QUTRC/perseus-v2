final: prev:
let
  individualPackages = individualFinal: individualPrev: {
    livox-ros-driver2 = individualFinal.callPackage ./livox-ros-driver2 { };
    fast-lio = individualFinal.callPackage ./fast-lio { };
    ndt-omp-ros2 = individualFinal.callPackage ./ndt-omp-ros2 { };
    vikit-common = individualFinal.callPackage ./vikit-common { };
    vikit-ros = individualFinal.callPackage ./vikit-ros { };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
  (import ./lidarslam-ros2/overlay.nix)
] final prev
