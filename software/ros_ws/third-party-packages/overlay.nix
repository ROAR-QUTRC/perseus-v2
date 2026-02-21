final: prev:
let
  individualPackages = individualFinal: individualPrev: {
    ortools = individualFinal.callPackage ./ortools { };
    livox-ros-driver2 = individualFinal.callPackage ./livox-ros-driver2 { };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
] final prev
