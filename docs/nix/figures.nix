{
  stdenv,
  lib,
  writeShellScript,
  drawio,
  parallel,
  xvfb-run,
}:
# Partly a ripoff of the drawio-headless package at:
# https://github.com/NixOS/nixpkgs/blob/nixpkgs-unstable/pkgs/applications/graphics/drawio/headless.nix
# Except highly specialised for our use case, and with no --auto-display because that breaks everything somehow.
# With that flag, xvfb-run doesn't work - it doesn't actually run any commands.
# Perhaps because it's in a Wayland environment not X11, and thus there's no pre-existing X server to talk to?
# -a works, but supposedly it's deprecated. Very strange. Running it with no flags works too, so that's what we do.
let
  output-dir = "generated";

  figures = stdenv.mkDerivation {
    name = "roar-docs-figures";
    src = lib.cleanSource ../figures-source;
    # after making the temp output dir, this finds all .drawio files in the source directory,
    # and parallel processes them using parallel with draw-io-convert, all run through xvfb-run so electron is happy
    buildPhase = ''
      mkdir ${output-dir}
      find ${../figures-source} -type f -name "*.drawio" -print0 | ${lib.getExe xvfb-run} ${lib.getExe parallel} -0 -j $(nproc) ${draw-io-convert}
    '';
    installPhase = ''
      mkdir -p $out
      cp -a ${output-dir}/. $out
    '';
  };

  # script to convert .drawio files to .svg in both light and dark themes
  # note that there's a whole bunch of cache/dbus errors: since they don't stop draw.io from running,
  # we can just ignore them!
  draw-io-convert = writeShellScript "draw-io-convert" ''
    set -e

    BASENAME=$(basename "$1")
    OUTPUT_BASE="${output-dir}"/"$BASENAME"

    convert() {
      INPUT_FILE="$1"
      OUTPUT_FILE="$2"
      shift 2
      ${lib.getExe drawio} -x "$INPUT_FILE" -o "$OUTPUT_FILE" --disable-gpu "$@" 
    }

    # Electron really wants a configuration directory to not die with:
    # "Error: Failed to get 'appData' path"
    # so we give it some temp dir as XDG_CONFIG_HOME
    tmpdir=$(mktemp -d)

    function cleanup {
      rm -rf "$tmpdir"
    }
    trap cleanup EXIT

    # Drawio needs to run in a virtual X session, because Electron
    # refuses to work and dies with an unhelpful error message otherwise:
    # "The futex facility returned an unexpected error code."
    export XDG_CONFIG_HOME="$tmpdir"

    convert "$1" "$OUTPUT_BASE"".svg" --svg-theme light
    convert "$1" "$OUTPUT_BASE""-dark.svg" --svg-theme dark
  '';
in
figures
