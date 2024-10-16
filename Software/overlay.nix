final: prev:
let
  shared = (import ./shared/overlay.nix);
  native = (import ./native/overlay.nix);
  ros_ws = (import ./ros_ws/overlay.nix);
  composed = prev.lib.composeManyExtensions [
    shared
    native
    ros_ws
  ] final prev;
in
composed
// {
  sharedDevPackages = (builtins.intersectAttrs (shared null null) final);
  nativeDevPackages = (builtins.intersectAttrs (native null null) final);
  # rosDevPackages provided by its ordinary overlay
}
