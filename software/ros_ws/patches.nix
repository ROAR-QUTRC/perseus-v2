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
