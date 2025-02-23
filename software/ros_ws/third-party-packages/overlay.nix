final: prev:
let
  individualPackages = prev: final: {
    ortools = final.callPackage ./ortools { };
  };
in
prev.lib.composeManyExtensions [
  individualPackages
  (import ./opennav-coverage/overlay.nix)
] final prev
