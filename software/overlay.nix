rosDistro: final: prev:
let
  shared = (import ./shared/overlay.nix);
  native = (import ./native/overlay.nix);
  ros_ws = (import ./ros_ws/overlay.nix rosDistro);
  composed = prev.lib.composeManyExtensions [
    (final: prev: { inherit cleanCmakeSource; })
    shared
    native
    ros_ws
  ] final prev;
  # utility function to provide a clean src with the cmake build directory (and typical build outputs) removed
  cleanCmakeSource =
    {
      src,
      name ? "src",
      ...
    }:
    prev.lib.cleanSourceWith {
      inherit name;
      src = prev.lib.cleanSource src;
      filter =
        path: type:
        let
          # strip out the absolute path prefix
          relativePath = prev.lib.removePrefix (toString src + "/") path;
        in
        !(prev.lib.hasPrefix "build/" relativePath);
    };
in
composed
// {
  sharedDevPackages = (builtins.intersectAttrs (shared null null) final);
  nativeDevPackages = (builtins.intersectAttrs (native null null) final);
  # rosDevPackages provided by its ordinary overlay
}
