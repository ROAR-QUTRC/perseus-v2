let
  # --- CONFIGURATION ---
  rosDistro = "jazzy";

  # --- FLAKE INPUTS ---
  lock = builtins.fromJSON (builtins.readFile ./../flake.lock);
  nixpkgs = fetchTarball {
    url =
      lock.nodes.nixpkgs.locked.url
        or "https://github.com/NixOS/nixpkgs/archive/${lock.nodes.nixpkgs.locked.rev}.tar.gz";
    sha256 = lock.nodes.nixpkgs.locked.narHash;
  };
  nix-ros-overlay = fetchTarball {
    url =
      lock.nodes.nix-ros-overlay.locked.url
        or "https://github.com/lopsided98/nix-ros-overlay/archive/${lock.nodes.nix-ros-overlay.locked.rev}.tar.gz";
    sha256 = lock.nodes.nix-ros-overlay.locked.narHash;
  };
  nix-ros-workspace = fetchTarball {
    url =
      lock.nodes.nix-ros-workspace.locked.url
        or "https://github.com/NixOS/nixpkgs/archive/${lock.nodes.nix-ros-workspace.locked.rev}.tar.gz";
    sha256 = lock.nodes.nix-ros-workspace.locked.narHash;
  };
  # --- OUTPUT PACKAGES ---
  pkgs = import nixpkgs {
    overlays = [
      (import (nix-ros-overlay + "/overlay.nix"))
      (import nix-ros-workspace { }).overlay
      (import ./overlay.nix rosDistro)
    ];
  };
in
pkgs
