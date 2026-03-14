final: prev:
let
  individualPackages = individualFinal: individualPrev: {
    livox-ros-driver2 = individualFinal.callPackage ./livox-ros-driver2 { };
    fast-lio = individualFinal.callPackage ./fast-lio { };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
] final prev
