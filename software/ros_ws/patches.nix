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
    # rosbridge incorrectly depends on the bson package instead of pymongo which is what it actually needs
    # (pymongo provides its own bson implementation, which behaves differently to the bson package)
    rosbridge-library = rosPrev.rosbridge-library.overrideAttrs (
      {
        propagatedBuildInputs ? [ ],
        ...
      }:
      let
        filteredPropagatedBuildInputs = rosFinal.lib.remove rosPrev.python3Packages.bson propagatedBuildInputs;
      in
      {
        propagatedBuildInputs = filteredPropagatedBuildInputs ++ [ rosPrev.python3Packages.pymongo ];
      }
    );
  };
in
{
  rosPackages = prev.rosPackages // {
    # we need to use overrideScope and an overlay to apply the changes
    # so that they propagate properly
    ${rosDistro} = prev.rosPackages.${rosDistro}.overrideScope rosOverlay;
  };
}
