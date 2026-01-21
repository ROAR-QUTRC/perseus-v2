final: prev:
let
  individualPackages = prev: final: {
    ortools = final.callPackage ./ortools { };
    livox-ros-driver2 = final.callPackage ./livox-ros-driver2 { };
    xtensor-stable = final.callPackage ./xtensor-stable { };
    xtl-stable = final.callPackage ./xtl-stable { };
    gz-msgs-vendor = final.callPackage ./gz-msgs-vendor { };
    gz-transport-vendor = final.callPackage ./gz-transport-vendor { };
    ros-gz-bridge = final.callPackage ./ros-gz-bridge { };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
] final prev
