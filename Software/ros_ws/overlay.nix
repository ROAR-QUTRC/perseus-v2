final: prev:
let
  # ros distro selection
  rosDistro = "humble";
  ros = prev.rosPackages.${rosDistro};

  # individual package overrides/fixes
  joint-state-publisher-gui = ros.joint-state-publisher-gui.overrideAttrs (
    {
      nativeBuildInputs ? [ ],
      qtWrapperArgs ? [ ],
      postFixup ? "",
      ...
    }:
    {
      dontWrapQtApps = false;
      nativeBuildInputs = nativeBuildInputs ++ [ prev.qt5.wrapQtAppsHook ];
      postFixup =
        postFixup
        + ''
          wrapQtApp "$out/lib/joint_state_publisher_gui/joint_state_publisher_gui"
        '';
    }
  );
in
{
  buildColconPackage = ros.callPackage ./build-colcon-package {
    inherit rosDistro;
    rosVersion = 2;
  };
  # mini package which puts COLCON_IGNORE in the output result folder
  # allows colcon build of workspace after run nix build
  colcon-ignore = prev.stdenv.mkDerivation rec {
    dontUnpack = true;
    name = "colcon-ignore";
    installPhase = ''
      mkdir -p $out
      touch $out/COLCON_IGNORE
    '';
  };

  ros =
    ros
    # ros packaging fixes
    // {
      inherit joint-state-publisher-gui;
    }
    # add workspace packages
    // ros.overrideScope (import ./nix-packages/overlay.nix);
}
