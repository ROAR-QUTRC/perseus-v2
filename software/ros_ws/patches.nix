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
    fields2cover =
      let
        nlohmann_json = final.nlohmann_json.overrideAttrs ({
          src = final.fetchFromGitHub {
            owner = "nlohmann";
            repo = "json";
            rev = "v3.10.5";
            sha256 = "sha256-DTsZrdB9GcaNkx7ZKxcgCA3A9ShM5icSF0xyGguJNbk=";
          };
          doCheck = false;
        });
        tinyxml-2 = final.tinyxml-2.overrideAttrs ({
          src = final.fetchFromGitHub {
            owner = "leethomason";
            repo = "tinyxml2";
            rev = "c2d30872e20621955ca7feb9168bad996d591a19";
            sha256 = "sha256-Gn4d6v7p60XRam2wclaSFAiAxmNgKAKPxRCEmcMtJIE=";
          };
        });
        steering-functions = final.stdenv.mkDerivation ({
          name = "steering-functions";
          src = final.fetchFromGitHub {
            owner = "Fields2Cover";
            repo = "steering_functions";
            rev = "33fc010017efa1ef2c8a2d4779fcda94b4b30d20";
            sha256 = "sha256-IaqvFWQFgU6yGOrvCrz7c7TEH4+vFf34iZ5qBZ1yMUw=";
          };
          nativeBuildInputs = with final; [
            cmake
          ];
          propagatedBuildInputs = with final; [
            eigen
          ];

          patches = [
            ./patches/fields2cover/steering-functions.patch
          ];
        });
        spline = final.stdenv.mkDerivation {
          name = "spline";
          # no version!

          src = final.fetchFromGitHub {
            owner = "Fields2Cover";
            repo = "spline";
            rev = "1b5d4bad29082997076b264de84ca6d46c2ae6ab";
            sha256 = "sha256-nWaanoqOVzDlxbfWFjT4j6ZQtp76E9tVbNVvx1R2dlM=";
          };

          nativeBuildInputs = with final; [
            cmake
          ];

          patches = [
            ./patches/fields2cover/spline.patch
          ];

          meta = {
            description = "c++ cubic spline library";
            license = with final.lib.licenses; [ gpl2 ];
          };
        };
        matplotlib-cpp = final.stdenv.mkDerivation {
          name = "matplotlib-cpp";
          # also no version!
          src = final.fetchFromGitHub {
            owner = "Fields2Cover";
            repo = "matplotlib-cpp";
            rev = "75c15d0c907a4b68bca5ef97032302bd14ccab8e";
            sha256 = "sha256-6rUm40oJaA8V9z1lj3HIC/TdZAp9GGEUPC0OvdESOMI=";
          };

          nativeBuildInputs = with final; [
            cmake
          ];
          propagatedBuildInputs = with final; [
            python3
            python3Packages.matplotlib
          ];

          patches = [
            ./patches/fields2cover/matplotlib-cpp.patch
          ];
        };
      in
      rosPrev.fields2cover.overrideAttrs (
        {
          propagatedBuildInputs ? [ ],
          patches ? [ ],
          postPatch ? "",
          ...
        }:
        {
          version = "1.2.1";
          src = final.fetchFromGitHub {
            owner = "ros2-gbp";
            repo = "fields2cover-release";
            rev = "upstream/1.2.1";
            sha256 = "sha256-g8LitJGD3iceBB2j10jdcNQWc3qE3JoSKe6oYsDR/CU=";
          };
          propagatedBuildInputs =
            with rosFinal;
            with final;
            [
              boost
              eigen
              gdal
              geos
              git
              gtest
              python3
              python3Packages.tkinter
              swig
              tbb_2021_11
              nlohmann_json
              tinyxml-2
              steering-functions
              spline
              matplotlib-cpp
            ];
          buildInputs = with final; [ or-tools ];
          patches = patches ++ [ ./patches/fields2cover/fields2cover.patch ];
          postPatch = ''
            ${postPatch}
            find . -type f -exec sed -i -e "s,gdal/,,g" {} \;
          '';
        }
      );

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

    librealsense2 = rosPrev.librealsense2.overrideAttrs (
      {
        cmakeFlags ? [ ],
        ...
      }:
      {
        cmakeFlags = cmakeFlags ++ [
          "-DCHECK_FOR_UPDATES=OFF"
          "-DBUILD_GRAPHICAL_EXAMPLES=OFF"
        ];
      }
    );
    rtabmap = rosPrev.rtabmap.overrideAttrs (
      {
        cmakeFlags ? [ ],
        ...
      }:
      {
        cmakeFlags = cmakeFlags ++ [
          "-DWITH_QT=OFF"
        ];
      }
    );
    fields2cover =
      let
        nlohmann_json = final.nlohmann_json.overrideAttrs ({
          src = final.fetchFromGitHub {
            owner = "nlohmann";
            repo = "json";
            rev = "v3.10.5";
            sha256 = "sha256-DTsZrdB9GcaNkx7ZKxcgCA3A9ShM5icSF0xyGguJNbk=";
          };
          doCheck = false;
        });
        tinyxml-2 = final.tinyxml-2.overrideAttrs ({
          src = final.fetchFromGitHub {
            owner = "leethomason";
            repo = "tinyxml2";
            rev = "c2d30872e20621955ca7feb9168bad996d591a19";
            sha256 = "sha256-Gn4d6v7p60XRam2wclaSFAiAxmNgKAKPxRCEmcMtJIE=";
          };
        });
        steering-functions = final.stdenv.mkDerivation ({
          name = "steering-functions";
          src = final.fetchFromGitHub {
            owner = "Fields2Cover";
            repo = "steering_functions";
            rev = "33fc010017efa1ef2c8a2d4779fcda94b4b30d20";
            sha256 = "sha256-IaqvFWQFgU6yGOrvCrz7c7TEH4+vFf34iZ5qBZ1yMUw=";
          };
          nativeBuildInputs = with final; [
            cmake
          ];
          propagatedBuildInputs = with final; [
            eigen
          ];

          patches = [
            ./patches/fields2cover/steering-functions.patch
          ];
        });
        spline = final.stdenv.mkDerivation {
          name = "spline";
          # no version!

          src = final.fetchFromGitHub {
            owner = "Fields2Cover";
            repo = "spline";
            rev = "1b5d4bad29082997076b264de84ca6d46c2ae6ab";
            sha256 = "sha256-nWaanoqOVzDlxbfWFjT4j6ZQtp76E9tVbNVvx1R2dlM=";
          };

          nativeBuildInputs = with final; [
            cmake
          ];

          patches = [
            ./patches/fields2cover/spline.patch
          ];

          meta = {
            description = "c++ cubic spline library";
            license = with final.lib.licenses; [ gpl2 ];
          };
        };
        matplotlib-cpp = final.stdenv.mkDerivation {
          name = "matplotlib-cpp";
          # also no version!
          src = final.fetchFromGitHub {
            owner = "Fields2Cover";
            repo = "matplotlib-cpp";
            rev = "75c15d0c907a4b68bca5ef97032302bd14ccab8e";
            sha256 = "sha256-6rUm40oJaA8V9z1lj3HIC/TdZAp9GGEUPC0OvdESOMI=";
          };

          nativeBuildInputs = with final; [
            cmake
          ];
          propagatedBuildInputs = with final; [
            python3
            python3Packages.matplotlib
          ];

          patches = [
            ./patches/fields2cover/matplotlib-cpp.patch
          ];
        };
      in
      rosPrev.fields2cover.overrideAttrs (
        {
          propagatedBuildInputs ? [ ],
          patches ? [ ],
          postPatch ? "",
          ...
        }:
        {
          version = "1.2.1";
          src = final.fetchFromGitHub {
            owner = "ros2-gbp";
            repo = "fields2cover-release";
            rev = "upstream/1.2.1";
            sha256 = "sha256-g8LitJGD3iceBB2j10jdcNQWc3qE3JoSKe6oYsDR/CU=";
          };
          propagatedBuildInputs =
            with rosFinal;
            with final;
            [
              boost
              eigen
              gdal
              geos
              git
              gtest
              python3
              python3Packages.tkinter
              swig
              tbb_2021_11
              nlohmann_json
              tinyxml-2
              steering-functions
              spline
              matplotlib-cpp
            ];
          buildInputs = with final; [ or-tools ];
          patches = patches ++ [ ./patches/fields2cover/fields2cover.patch ];
          postPatch = ''
            ${postPatch}
            find . -type f -exec sed -i -e "s,gdal/,,g" {} \;
          '';
        }
      );

    opennav-coverage = rosPrev.opennav-coverage.overrideAttrs (
      {
        propagatedBuildInputs ? [ ],
        ...
      }:
      {
        propagatedBuildInputs = propagatedBuildInputs ++ [ rosFinal.visualization-msgs ];
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
          ...
        }:
        rec {
          version = "1.14.0-alpha.1";
          src = final.fetchFromGitLab {
            domain = "gitlab.freedesktop.org";
            owner = "gstreamer";
            repo = "gst-plugins-rs";
            rev = "9b0aa9c710874c07f37cf37f964e9942f9d89640";
            # hash = "sha256-4qySdwd0Zo3HT7hm9+x9BjhuxO2Y4TBcDwWprnIJq54=";
            hash = "sha256-WNuQEu77vr4+cYlY7vrhVRqIpgc6v1PiqLlhUHDZ4SI=";
            # TODO: temporary workaround for case-insensitivity problems with color-name crate - https://github.com/annymosse/color-name/pull/2
            postFetch = ''
              sedSearch="$(cat <<\EOF | sed -ze 's/\n/\\n/g'
              \[\[package\]\]
              name = "color-name"
              version = "\([^"\n]*\)"
              source = "registry+https://github.com/rust-lang/crates.io-index"
              checksum = "[^"\n]*"
              EOF
              )"
              sedReplace="$(cat <<\EOF | sed -ze 's/\n/\\n/g'
              [[package]]
              name = "color-name"
              version = "\1"
              source = "git+https://github.com/lilyinstarlight/color-name#cac0ed5b7d2e0682c08c9bfd13089d5494e81b9a"
              EOF
              )"
              sed -i -ze "s|$sedSearch|$sedReplace|g" $out/Cargo.lock
            '';
          };
          cargoDeps =
            with final;
            rustPlatform.fetchCargoVendor {
              inherit src;
              name = final.gst_all_1.gst-plugins-rs.name;
              hash = "sha256-HHMZ8kBA8g2cjYxrZnEsX33xC3glyKius3jxP84H+PY=";
            };
        }
      )

      ).override
        ({
          plugins = [
            "rtp"
            "webrtc"
          ];
        });
  };
}
