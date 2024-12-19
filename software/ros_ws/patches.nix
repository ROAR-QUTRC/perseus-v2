rosDistro: final: prev:
let
  rosOverlay = rosFinal: rosPrev: {
    # --- GUI patches ---
    joint-state-publisher-gui = rosPrev.joint-state-publisher-gui.overrideAttrs (
      {
        nativeBuildInputs ? [ ],
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
  };
in
{
  rosPackages = prev.rosPackages // {
    # we need to use overrideScope and an overlay to apply the changes
    # so that they propagate properly (eg most of nav2 is dependent on nav-2d-utils)
    ${rosDistro} = prev.rosPackages.${rosDistro}.overrideScope rosOverlay;
  };
}
