{
  rosDistro,
  mkShell,
  env,
  uv,
  drawio,
}:
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
