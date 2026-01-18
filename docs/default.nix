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
  unstable,
  graphviz,
  # ros version to link to with intersphinx, with default
  rosDistro ? "jazzy",
}:
let
  # mostly taken from uv2nix docs https://adisbladis.github.io/uv2nix/usage/hello-world.html
  # Load uv workspace from the workspace root
  # note that moving the python dependencies to ./pyproject ensures that updating _docs_ doesn't cause a workspace rebuild
  workspace = uv2nix.lib.workspace.loadWorkspace { workspaceRoot = ./pyproject; };

  # Create package overlay from workspace.
  overlay = workspace.mkPyprojectOverlay { sourcePreference = "wheel"; };

  # Construct package set, using base package set from pyproject.nix builders
  # for details see https://pyproject-nix.github.io/pyproject.nix/build.html#resolving
  # Unlike an ordinary Nix package set, this is not directly usable.
  # Since Python packages can have different run- and build-time dependencies which may or may not need to propagate,
  # pyproject.nix uses passthru attributes to specify all of this.
  # This also means that you need to a *builder* to actually "render" the output package set and make it usable (typically with a virtualenv),
  # hence the comparatively large number of steps
  pythonPkgSet = (callPackage pyproject-nix.build.packages { python = python312; }).overrideScope (
    lib.composeManyExtensions [
      pyproject-build-systems.overlays.default
      overlay
    ]
  );

  # Resolve a virtualenv with everything python in it
  pyEnv = pythonPkgSet.mkVirtualEnv "roar-docs-py-env" workspace.deps.default;

  # add generation tools to the final environment so we can generate code docs
  # next, use the resolved venv and add the other tools we need to generate the code docs -
  # this is the environment that will be passed as a dependency to the build and dev shell derivations
  env = symlinkJoin {
    name = "roar-docs-env";
    paths = [
      pyEnv
      unstable.doxygen12
      graphviz
    ];
  };

  # define all the passthru attributes
  # Defining them here in the `let` block allows convenient recursive evaluation
  shell = callPackage (import ./nix/shell.nix) { inherit rosDistro env; };
  compressed = callPackage (import ./nix/compressed.nix) { inherit docs; };
  decompress = callPackage (import ./nix/decompress.nix) { };
  cd-docs-source = callPackage (import ./nix/cd-docs-source.nix) { };
  figures = callPackage (import ./nix/figures.nix) { };
  setup = callPackage (import ./nix/setup.nix) { inherit cd-docs-source figures; };

  # create derivation which builds the docs
  docs = stdenvNoCC.mkDerivation {
    name = "roar-docs";
    buildInputs = [ env ];
    # filtering the source ensures that we don't have (too many) unnecessary extra rebuilds when something changes even though it doesn't affect the output
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

    # use passthru attributes for scripts, other build targets (eg figures and compressed), and the build env
    passthru = {
      inherit
        env
        shell
        compressed
        decompress
        figures
        setup
        ;
    };

    # this needs to be set as an environment variable (just passing it as part of the set to mkDerivation)
    # for intersphinx config to be sure to use the correct file
    ROS_DISTRO = rosDistro;

    # make needs to be run from the docs directory, but we need the whole tree for Doxygen generation
    # (hence source being the whole project)

    buildPhase = ''
      cd docs

      # copy prebuilt figures into place
      mkdir -p source/generated
      cp -a ${figures}/. source/generated

      # needed to prevent /homeless-shelter from being created
      export HOME=$PWD

      # make the entire directory writable - allows us to modify the files during the Sphinx build
      # (mainly for figures)
      chmod -R +w .

      # finally build the docs
      make html
    '';
    # install the docs to $out/html
    installPhase = ''
      mkdir -p $out/html
      cp -a ./build/html/. $out/html
    '';
  };
in
docs
