{
  nix-ros-overlay ? builtins.fetchTarball "https://github.com/lopsided98/nix-ros-overlay/archive/master.tar.gz",
}:
let
  applyDistroOverlay =
    rosOverlay: rosPackages:
    rosPackages
    // builtins.mapAttrs (
      rosDistro: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
    ) rosPackages;
  rosDistroOverlays = final: prev: {
    # Apply the overlay to multiple ROS distributions
    rosPackages = applyDistroOverlay (import ./overlay.nix) prev.rosPackages;
  };
in
import nix-ros-overlay {
  overlays = [ rosDistroOverlays ];
}
