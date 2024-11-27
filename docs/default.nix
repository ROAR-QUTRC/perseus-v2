{
  # standard nix tooling
  lib,
  stdenvNoCC,
  callPackage,
  python312,
  symlinkJoin,
  # python tooling
  pyproject-nix,
  uv2nix,
  pyproject-build-systems,
  # non-python packages to build docs
  doxygen,
  graphviz,
  # ros version to link to with intersphinx - defaults to humble
  rosDistro ? "humble",
}:
let
  # mostly taken from uv2nix docs https://adisbladis.github.io/uv2nix/usage/hello-world.html
  # Load uv workspace from the workspace root
  # note that moving the python dependencies to ./pyproject ensures that updating _docs_ doesn't cause a workspace rebuild
  workspace = uv2nix.lib.workspace.loadWorkspace { workspaceRoot = ./pyproject; };

  # Create package overlay from workspace.
  overlay = workspace.mkPyprojectOverlay { sourcePreference = "wheel"; };

  # Construct package set, using base package set from pyproject.nix builders
  pythonPkgSet = (callPackage pyproject-nix.build.packages { python = python312; }).overrideScope (
    lib.composeManyExtensions [
      pyproject-build-systems.overlays.default
      overlay
    ]
  );
  pyEnv = pythonPkgSet.mkVirtualEnv "roar-docs-py-env" workspace.deps.default;
  # add generation tools to the final environment so we can generate code docs
  env = symlinkJoin {
    name = "roar-docs-env";
    paths = [
      pyEnv
      doxygen
      graphviz
    ];
  };

  # create derivation which builds the docs
  docs = stdenvNoCC.mkDerivation {
    name = "roar-docs";
    buildInputs = [ env ];
    src = lib.cleanSourceWith {
      name = "roar-docs-src";
      src = lib.cleanSource ./..;
      filter =
        name: type:
        let
          relativePath = lib.removePrefix (toString ./..) name;
        in
        !(
          # filter out build directory
          (lib.hasInfix "build/" relativePath)
          # filter out generated files
          || (lib.hasInfix "generated/" relativePath)
        );
    };

    passthru = {
      inherit
        env
        shell
        compressed
        decompress
        figures
        fetch-inventories
        setup
        ;
    };

    ROS_DISTRO = rosDistro; # for intersphinx config

    # make needs to be run from the docs directory, but we need the whole tree for Doxygen generation
    # (hence source being the whole project)
    # note that (although they should not be present anyway) the program_listing files are removed to
    # doubly ensure that there are no source code leaks
    buildPhase = ''
      cd docs

      # copy prebuilt figures into place
      mkdir -p source/generated
      cp -a ${figures}/. source/generated

      # needed to prevent /homeless-shelter from being created
      export HOME=$PWD

      # this needed because Sphinx does a dumb thing and overrides the "current year" in copyright with SOURCE_DATE_EPOCH
      # see https://github.com/sphinx-doc/sphinx/issues/3451#issuecomment-877801728
      unset SOURCE_DATE_EPOCH
      # make the directory writable - allows us to modify the files during the Sphinx build
      chmod -R +w .
      # finally build the docs
      make html

      # failsafe to *ensure* that program listings are removed - we REALLY don't want to leak source code
      find -type f -name '*program_listing*' -delete
    '';
    # install the docs to $out/html
    installPhase = ''
      mkdir -p $out/html
      cp -a ./build/html/. $out/html
    '';
  };

  shell = callPackage (import ./nix/shell.nix) { inherit rosDistro env; };
  compressed = callPackage (import ./nix/compressed.nix) { inherit docs; };
  decompress = callPackage (import ./nix/decompress.nix) { };
  cd-docs-source = callPackage (import ./nix/cd-docs-source.nix) { };
  figures = callPackage (import ./nix/figures.nix) { };
  fetch-inventories = callPackage (import ./nix/fetch-inventories.nix) {
    inherit rosDistro cd-docs-source;
  };
  setup = callPackage (import ./nix/setup.nix) { inherit cd-docs-source figures; };
in
docs
