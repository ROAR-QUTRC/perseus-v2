final: prev:
let
  individualPackages = prev: final: {
    ortools = final.callPackage ./ortools { };
    livox-ros-driver2 = final.callPackage ./livox-ros-driver2 { };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
] final prev
