final: prev:
let
  joint-state-publisher-gui = prev.ros.joint-state-publisher-gui.overrideAttrs
    ({ nativeBuildInputs ? [ ], qtWrapperArgs ? [ ], postFixup ? "", ... }: {
      dontWrapQtApps = false;
      nativeBuildInputs = nativeBuildInputs ++ [ prev.qt5.wrapQtAppsHook ];
      postFixup = postFixup + ''
        wrapQtApp "$out/lib/joint_state_publisher_gui/joint_state_publisher_gui"
      '';
    });
in { ros = prev.ros // { inherit joint-state-publisher-gui; }; }
