rosDistro: final: prev:
let
  shared = (import ./shared/overlay.nix);
  native = (import ./native/overlay.nix);
  ros_ws = (import ./ros_ws/overlay.nix rosDistro);
  scripts = (import ./scripts/overlay.nix);
  composed = prev.lib.composeManyExtensions [
    (final: prev: { inherit cleanCmakeSource; })
    shared
    native
    ros_ws
    scripts
  ] final prev;
  # utility function to provide a clean src with the cmake build directory (and typical build outputs) removed
  cleanCmakeSource =
    {
      src,
      name ? "src",
      ...
    }:
    prev.lib.cleanSourceWith {
      inherit name;
      src = prev.lib.cleanSource src;
      filter =
        path: type:
        let
          # strip out the absolute path prefix
          relativePath = prev.lib.removePrefix (toString src) path;
        in
        !(prev.lib.hasPrefix "build/" relativePath);
    };

  buildPackageCoverage =
    pkg:
    let
      coverage = pkg.overrideAttrs (
        {
          cmakeFlags ? [ ],
          pname,
          version ? "",
          ...
        }:
        {
          name = if version != "" then "${pname}-${version}-coverage" else "${pname}-coverage";
          cmakeBuildType = "Debug";
          cmakeFlags = cmakeFlags ++ [
            "-DBUILD_TESTING=ON"
            "-DTEST_COVERAGE=ON"
          ];
          # Use gcovr to generate test coverage reports.
          # Testing (which will build the coverage information) is done in the checkPhase.
          # The test source code directories are excluded as they skew the coverage results
          # (but they *also* need to be built with coverage, otherwise header files are not properly included in the reports).
          installPhase = ''
            OUTPUT_DIR=$out/share/test-coverage/${pname}
            mkdir -p "$OUTPUT_DIR"
            ${final.unstable.gcovr}/bin/gcovr -r .. -e ../tests -e ../test --html-details -o "$OUTPUT_DIR"/index.html
          '';
        }
      );
    in
    coverage;
  buildPackageCoverage' =
    pkgs:
    let
      # Filter if the package is built with ament_python,
      # doesn't contain a `src` attribute, or the `src` attribute doesn't contain a `src/` directory
      # Should be good enough for this project, since we only have C++ and Python packages
      isCppPackage =
        pkg:
        ((pkg.buildType or "") != "ament_python")
        && (builtins.hasAttr "src" pkg)
        && (builtins.pathExists (pkg.src + "/src"));
      # input set needs to be converted to a list so we can filter it
      cppPkgs = builtins.filter isCppPackage (builtins.attrValues pkgs);
      # finally, map each package to its coverage build
      coverageList = builtins.map buildPackageCoverage cppPkgs;
    in
    # and use buildEnv to merge all the outputs into a single file tree
    final.buildEnv {
      name = "coverage";
      paths = coverageList;
    };
in
composed
// {
  sharedDevPackages = (builtins.intersectAttrs (shared null null) final);
  nativeDevPackages = (builtins.intersectAttrs (native null null) final);
  # ros dev package sets provided in the ros overlay files
  coverage = buildPackageCoverage' (
    final.sharedDevPackages // final.nativeDevPackages // final.ros.devPackages
  );
}
