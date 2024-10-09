# WARNING! THIS IS VERY BROKEN RIGHT NOW.
# colcon outputs a local_setup.* and setup.* files, which cause merge errors when used in a workspace.
# The solution is probably to set up an install phase so that files get moved to the correct location
# Don't have the time to do that right now. - James N
{
  stdenv,
  lib,
  colcon,
  pythonPackages,
  rosDistro,
  rosVersion,
}:
{
  doCheck ? false, # TODO: implement colcon test phase
  # nixpkgs requires that either dontWrapQtApps is set or wrapQtAppsHook is added
  # to nativeBuildInputs if a package depends on Qt5. This is difficult to achieve
  # with auto-generated packages, so we just always disable wrapping except for
  # packages that are overridden in distro-overlay.nix. This means some Qt5
  # applications are broken, but allows all libraries that depend on Qt5 to build
  # correctly.
  dontWrapQtApps ? true,
  nativeBuildInputs ? [ ],
  CXXFLAGS ? "",
  postFixup ? "",
  passthru ? { },
  separateDebugInfo ? true,
  ...
}@args:

stdenv.mkDerivation (
  args
  // {
    inherit doCheck dontWrapQtApps separateDebugInfo;

    # Disable warnings that cause "Log limit exceeded" errors on Hydra in lots of
    # packages that use Eigen
    CXXFLAGS = CXXFLAGS + " -Wno-deprecated-declarations -Wno-deprecated-copy";

    nativeBuildInputs = nativeBuildInputs ++ [ colcon ]; # python3Packages.colcon-core ];

    dontUseCmakeConfigure = true;
    buildPhase = ''
      runHook preBuild
      colcon --log-level info build --install-base "$out" --merge-install
      runHook postBuild
    '';

    passthru = passthru // {
      rosPackage = true;
      inherit rosDistro rosVersion;
    };
  }
)
