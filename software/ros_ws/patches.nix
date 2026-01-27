rosDistro: final: prev:
let
  rosOverlay = rosFinal: rosPrev: {
    # Fix conflict between ament-cmake-vendor-package and ament-cmake-vendor-package-wrapped.
    # nix-ros-overlay defines two versions: base (path patches only) and wrapped (adds
    # ament_vendor_wrapper.cmake for Nix sandbox). Both end up in the devShell's buildEnv
    # via different dependency paths, causing file conflicts.
    #
    # The conflicting files (cmake configs, setup scripts) are functionally identical
    # between both versions - the wrapper only adds functionality, it doesn't change
    # existing files. Therefore ignoreCollisions is safe here.
    #
    # A proper upstream fix would ensure only one version is used, but that requires
    # changes to nix-ros-overlay's ros2-overlay.nix. We can't override the packages
    # directly here due to circular dependencies in how they're defined.
    #
    # See: https://github.com/lopsided98/nix-ros-overlay ros2-overlay.nix
    buildEnv =
      args:
      rosPrev.buildEnv (
        args
        // {
          ignoreCollisions = true;
        }
      );

    # # --- GUI patches ---
    # fields2cover =
    #   let
    #     nlohmann_json = final.nlohmann_json.overrideAttrs ({
    #       src = final.fetchFromGitHub {
    #         owner = "nlohmann";
    #         repo = "json";
    #         rev = "v3.10.5";
    #         sha256 = "sha256-DTsZrdB9GcaNkx7ZKxcgCA3A9ShM5icSF0xyGguJNbk=";
    #       };
    #       doCheck = false;
    #     });
    #     tinyxml-2 = final.tinyxml-2.overrideAttrs ({
    #       src = final.fetchFromGitHub {
    #         owner = "leethomason";
    #         repo = "tinyxml2";
    #         rev = "c2d30872e20621955ca7feb9168bad996d591a19";
    #         sha256 = "sha256-Gn4d6v7p60XRam2wclaSFAiAxmNgKAKPxRCEmcMtJIE=";
    #       };
    #     });
    #     steering-functions = final.stdenv.mkDerivation ({
    #       name = "steering-functions";
    #       src = final.fetchFromGitHub {
    #         owner = "Fields2Cover";
    #         repo = "steering_functions";
    #         rev = "33fc010017efa1ef2c8a2d4779fcda94b4b30d20";
    #         sha256 = "sha256-IaqvFWQFgU6yGOrvCrz7c7TEH4+vFf34iZ5qBZ1yMUw=";
    #       };
    #       nativeBuildInputs = with final; [
    #         cmake
    #       ];
    #       propagatedBuildInputs = with final; [
    #         eigen
    #       ];

    #       patches = [
    #         ./patches/fields2cover/steering-functions.patch
    #       ];
    #     });
    #     spline = final.stdenv.mkDerivation {
    #       name = "spline";
    #       # no version!

    #       src = final.fetchFromGitHub {
    #         owner = "Fields2Cover";
    #         repo = "spline";
    #         rev = "1b5d4bad29082997076b264de84ca6d46c2ae6ab";
    #         sha256 = "sha256-nWaanoqOVzDlxbfWFjT4j6ZQtp76E9tVbNVvx1R2dlM=";
    #       };

    #       nativeBuildInputs = with final; [
    #         cmake
    #       ];

    #       patches = [
    #         ./patches/fields2cover/spline.patch
    #       ];

    #       meta = {
    #         description = "c++ cubic spline library";
    #         license = with final.lib.licenses; [ gpl2 ];
    #       };
    #     };
    #     matplotlib-cpp = final.stdenv.mkDerivation {
    #       name = "matplotlib-cpp";
    #       # also no version!
    #       src = final.fetchFromGitHub {
    #         owner = "Fields2Cover";
    #         repo = "matplotlib-cpp";
    #         rev = "75c15d0c907a4b68bca5ef97032302bd14ccab8e";
    #         sha256 = "sha256-6rUm40oJaA8V9z1lj3HIC/TdZAp9GGEUPC0OvdESOMI=";
    #       };

    #       nativeBuildInputs = with final; [
    #         cmake
    #       ];
    #       propagatedBuildInputs = with final; [
    #         python3
    #         python3Packages.matplotlib
    #       ];

    #       patches = [
    #         ./patches/fields2cover/matplotlib-cpp.patch
    #       ];
    #     };
    #   in
    #   rosPrev.fields2cover.overrideAttrs (
    #     {
    #       patches ? [ ],
    #       postPatch ? "",
    #       ...
    #     }:
    #     {
    #       version = "1.2.1";
    #       src = final.fetchFromGitHub {
    #         owner = "ros2-gbp";
    #         repo = "fields2cover-release";
    #         rev = "upstream/1.2.1";
    #         sha256 = "sha256-g8LitJGD3iceBB2j10jdcNQWc3qE3JoSKe6oYsDR/CU=";
    #       };
    #       propagatedBuildInputs =
    #         with rosFinal;
    #         with final;
    #         [
    #           boost
    #           eigen
    #           gdal
    #           geos
    #           git
    #           gtest
    #           python3
    #           python3Packages.tkinter
    #           swig
    #           tbb_2021_11
    #           nlohmann_json
    #           tinyxml-2
    #           steering-functions
    #           spline
    #           matplotlib-cpp
    #         ];
    #       buildInputs = with final; [ or-tools ];
    #       patches = patches ++ [ ./patches/fields2cover/fields2cover.patch ];
    #       postPatch = ''
    #         ${postPatch}
    #         find . -type f -exec sed -i -e "s,gdal/,,g" {} \;
    #       '';
    #     }
    #   );

    opennav-coverage = rosPrev.opennav-coverage.overrideAttrs (
      {
        propagatedBuildInputs ? [ ],
        ...
      }:
      {
        propagatedBuildInputs = propagatedBuildInputs ++ [ rosFinal.visualization-msgs ];
      }
    );

    livox-ros-driver2 = rosPrev.livox-ros-driver2.overrideAttrs (
      {
        buildInputs ? [ ],
        patches ? [ ],
        ...
      }:
      {
        buildInputs = buildInputs ++ [ final.livox-sdk2 ];
        patches = patches ++ [
          ./patches/livox-ros-driver2/rename-files.patch
          ./patches/livox-ros-driver2/livox-ros-driver2.patch
        ];
      }
    );

    perseus-input = rosPrev.perseus-input.overrideAttrs (
      {
        propagatedBuildInputs ? [ ],
        ...
      }:
      {
        propagatedBuildInputs = final.lib.remove rosFinal.perseus-input-config propagatedBuildInputs;
      }
    );

    python-qt-binding = rosPrev.python-qt-binding.overrideAttrs (
      {
        propagatedBuildInputs ? [ ],
        ...
      }:
      {
        patches = [ ];
        propagatedBuildInputs =
          (final.lib.remove final.python3Packages.pyqt6-sip (
            final.lib.remove final.python3Packages.pyside6 propagatedBuildInputs
          ))
          ++ [
            final.python312Packages.pyside2
            final.python312Packages.shiboken2
          ];
      }
    );

    nav2-mppi-controller = rosPrev.nav2-mppi-controller.override {
      xtensor = final.ros.xtensor-stable;
    };

    gz-physics-vendor = rosPrev.gz-physics-vendor.overrideAttrs (
      {
        patches ? [ ],
        ...
      }:
      {
        patches = patches ++ [ ./patches/gz-physics-vendor/version.patch ];
      }
    );

    gz-rendering-vendor = rosPrev.gz-rendering-vendor.overrideAttrs (
      {
        patches ? [ ],
        ...
      }:
      {
        patches = patches ++ [ ./patches/gz-rendering-vendor/version.patch ];
      }
    );

    gz-msgs-vendor = rosPrev.gz-msgs-vendor.override {
      protobuf = final.oldPkgs.protobuf_28;
      python3Packages = prev.python3Packages // {
        protobuf = prev.python3Packages.protobuf.override {
          protobuf = final.oldPkgs.protobuf_28;
        };
      };
    };

    gz-transport-vendor = rosPrev.gz-transport-vendor.overrideAttrs (
      {
        patches ? [ ],
        ...
      }:
      {
        patches = patches ++ [ ./patches/gz-transport-vendor/version.patch ];
      }
    );

    gz-sim-vendor = rosPrev.gz-sim-vendor.overrideAttrs (
      {
        patches ? [ ],
        ...
      }:
      {
        patches = patches ++ [ ./patches/gz-sim-vendor/version.patch ];
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

  gst_all_1 = prev.gst_all_1 // {
    gst-plugins-rs =
      (prev.gst_all_1.gst-plugins-rs.overrideAttrs (
        {
          patches ? [ ],
          ...
        }:
        {
          version = "0.14.4";
          src = final.fetchFromGitLab {
            domain = "gitlab.freedesktop.org";
            owner = "gstreamer";
            repo = "gst-plugins-rs";
            rev = "0.14.4";
            # hash = "sha256-4qySdwd0Zo3HT7hm9+x9BjhuxO2Y4TBcDwWprnIJq54=";
            hash = "sha256-MZyYHMq6gFJkVxlrmeXUjOmRYsQBHj0848cnF+7mtbU=";
          };
          cargoDeps =
            with final;
            rustPlatform.fetchCargoVendor {
              inherit src;
              name = final.gst_all_1.gst-plugins-rs.name;
              hash = "sha256-T+fdu+Oe07Uf1YoRGYl2DMb1QgdSZVLwcOqH4bBNGXU=";
            };
          patches = builtins.elemAt patches 0;
        }
      )

      ).override
        {
          plugins = [
            "rtp"
            "webrtc"
          ];
        };
  };
  pcl = final.oldPkgs.pcl;
}
