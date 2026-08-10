{ pkg-config, mpi }:
final: prev: {
  mesh-msgs = final.callPackage ./mesh-msgs.nix { };
  mesh-msgs-conversions = final.callPackage ./mesh-msgs-conversions.nix { inherit pkg-config mpi; };
}
