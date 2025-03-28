final: prev:
let
  individualPackages = prev: final: {
    ortools = final.callPackage ./ortools { };
    livox-ros-driver2 = final.callPackage ./livox-ros-driver2 { };
    fast-lio = final.callPackage ./fast-lio { };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
] final prev
