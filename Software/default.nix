let
  nixpkgs = fetchTarball "https://github.com/NixOS/nixpkgs/tarball/nixos-24.05";
  nix-ros-overlay = fetchTarball "https://github.com/lopsided98/nix-ros-overlay/archive/master.tar.gz";
  nix-ros-workspace = fetchTarball "https://github.com/hacker1024/nix-ros-workspace/archive/master.tar.gz";
  pkgs = import nixpkgs {
    config = { };
    overlays = [
      (import (nix-ros-overlay + "/overlay.nix"))
      (import nix-ros-workspace { }).overlay
      (import ./overlay.nix)
    ];
  };
in
builtins.intersectAttrs ((import ./overlay.nix) null null) pkgs
