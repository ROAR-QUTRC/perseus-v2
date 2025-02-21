# packages/overlay.nix
final: prev: {
  groot2 = prev.callPackage ./groot2 { };
  livox-sdk2 = prev.callPackage ./livox-sdk2 { };
  livox-ros-driver2 = prev.callPackage ./livox-ros-driver2 {
    inherit (final) livox-sdk2;
  };
}
