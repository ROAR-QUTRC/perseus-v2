# packages/overlay.nix
final: prev: {
  groot2 = final.callPackage ./groot2 { };
  livox-sdk2 = final.callPackage ./livox-sdk2 { };
}
