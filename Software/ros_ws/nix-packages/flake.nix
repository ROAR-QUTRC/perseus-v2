
{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        applyDistroOverlay =
          rosOverlay: rosPackages:
          rosPackages
          // builtins.mapAttrs (
            rosDistro: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
          ) rosPackages;
        rosDistroOverlays = self: super: {
          # Apply the overlay to multiple ROS distributions
          rosPackages = applyDistroOverlay (import ./overlay.nix) super.rosPackages;
        };
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            rosDistroOverlays
          ];
        };

        rosDistro = "humble";

      in {
        legacyPackages = pkgs.rosPackages;
        packages = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        checks = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        devShells.default = pkgs.mkShell {
          name = "Example project";
          packages = [
            pkgs.colcon
            # ... other non-ROS packages
            (with pkgs.rosPackages.${rosDistro}; buildEnv {
              paths = [
                ros-core
                # ... other ROS packages
              ];
            })
          ];
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
