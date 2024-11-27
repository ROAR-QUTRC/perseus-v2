{
  rosDistro,
  mkShell,
  env,
  uv,
  drawio,
}:
# provide a dev shell environment for the docs - see pyproject docs:
# https://pyproject-nix.github.io/uv2nix/usage/hello-world.html
# not that since we're using it solely for dependency management,
# rather than developing a python project, we don't need an editable environment,
# so we can just reuse the build environment.
# Plus include uv for managing said deps and drawio for figures
let
  shell = mkShell {
    ROS_DISTRO = rosDistro;
    buildInputs = [
      env
      uv # uv added to manage python dependencies
      drawio
    ];
    shellHook = ''
      # Undo dependency propagation by nixpkgs.
      unset PYTHONPATH
    '';
  };
in
shell
