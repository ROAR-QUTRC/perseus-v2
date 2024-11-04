rosDistro: final: prev:
let
  rosPrev = prev.rosPackages.${rosDistro};

  # --- GUI patches ---
  joint-state-publisher-gui = rosPrev.joint-state-publisher-gui.overrideAttrs (
    {
      nativeBuildInputs ? [ ],
      postFixup ? "",
      ...
    }:
    {
      dontWrapQtApps = false;
      nativeBuildInputs = nativeBuildInputs ++ [ rosPrev.qt5.wrapQtAppsHook ];
      postFixup =
        postFixup
        + ''
          wrapQtApp "$out/lib/joint_state_publisher_gui/joint_state_publisher_gui"
        '';
    }
  );

  # --- nav2 patches ---
  # since this gets propagated and the error is in a header file, the error also gets propagated,
  # so we have to patch it properly
  nav-2d-utils = rosPrev.nav-2d-utils.overrideAttrs (
    {
      patches ? [ ],
      ...
    }:
    {
      patches = patches ++ [ ./patches/fix-nav-2d-utils.patch ];
    }
  );
  nav2-behaviors = rosPrev.nav2-behaviors.overrideAttrs (
    {
      CXXFLAGS ? "",
      ...
    }:
    {
      CXXFLAGS = CXXFLAGS + " -Wno-error=maybe-uninitialized";
    }
  );
  # same as with nav-2d-utils - it doesn't have any direct dependencies using its header file,
  # but it's good to patch it anyway
  nav2-constrained-smoother = rosPrev.nav2-constrained-smoother.overrideAttrs (
    {
      patches ? [ ],
      ...
    }:
    {
      patches = patches ++ [ ./patches/fix-nav2-constrained-smoother.patch ];
    }
  );
  nav2-mppi-controller = rosPrev.nav2-mppi-controller.overrideAttrs (
    {
      CXXFLAGS ? "",
      ...
    }:
    {
      # WARNING: This is probably extremely bad, but it makes it build.
      # This package builds on Ubuntu normally, but not in Nix with the same dependencies.
      # so, this makes it build.
      CXXFLAGS = CXXFLAGS + " -Wno-error=array-bounds";
    }
  );
  nav2-planner = rosPrev.nav2-planner.overrideAttrs (
    {
      CXXFLAGS ? "",
      ...
    }:
    {
      CXXFLAGS = CXXFLAGS + " -Wno-error=maybe-uninitialized";
    }
  );
  # for some reason overrideScope is not properly replacing nav-2d-utils as an input to this package,
  # so we have to do it manually
  nav2-smoother =
    (rosPrev.nav2-smoother.overrideAttrs (
      {
        CXXFLAGS ? "",
        ...
      }:
      {
        CXXFLAGS = CXXFLAGS + " -Wno-error=maybe-uninitialized";
      }
    )).override
      { inherit nav-2d-utils; };
  nav2-waypoint-follower = rosPrev.nav2-waypoint-follower.overrideAttrs (
    {
      CXXFLAGS ? "",
      ...
    }:
    {
      CXXFLAGS = CXXFLAGS + " -Wno-error=maybe-uninitialized";
    }
  );

  # output package set
  ros = {
    inherit
      joint-state-publisher-gui
      nav-2d-utils
      nav2-behaviors
      nav2-constrained-smoother
      nav2-mppi-controller
      nav2-planner
      nav2-smoother
      nav2-waypoint-follower
      ;
  };

  # we need to use overrideScope and an overlay to apply the changes
  # so that they propagate properly (eg most of nav2 is dependent on nav-2d-utils)
  rosOverlay = rosFinal: rosPrev: ros;
in
{
  rosPackages = prev.rosPackages // {
    ${rosDistro} = (rosPrev.overrideScope rosOverlay);
  };
}
