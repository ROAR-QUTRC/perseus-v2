{
  inputs,
  pkgs-unstable,
  rosDistro,
  self,
  ...
}:
[
  inputs.nixgl.overlay
  # TODO: Look into fixing nix-gl-host? Doesn't work on Intel hardware
  inputs.nix-gl-host.overlays.default
  # get the ros packages
  inputs.nix-ros-overlay.overlays.default
  # fix colcon (silence warnings, add extensions)
  (import ../software/ros_ws/colcon/overlay.nix)
  # add ros workspace functionality
  inputs.nix-ros-workspace.overlays.default
  # import ros workspace packages + fixes
  (import ../software/overlay.nix rosDistro)
  (import ../packages/overlay.nix)
  (final: prev: {
    inherit self; # add self access for hacks like nixGL
    # alias the output to pkgs.ros to make it easier to use
    ros = final.rosPackages.${rosDistro};
    # and add pkgs.unstable access
    unstable = pkgs-unstable;
  })
]
