{
  # standard nix tooling
  lib,
  stdenv,
  callPackage,
  python312,
  symlinkJoin,
  # python tooling
  pyproject-nix,
  uv2nix,
  doxygen,
  # shell env
  mkShell,
  uv,
  # compressed docs
  p7zip,
  writeShellScriptBin,
}:
let
  # mostly taken from uv2nix docs https://adisbladis.github.io/uv2nix/usage/hello-world.html
  # Load uv workspace from the workspace root
  # note that moving the python dependencies to ./pyproject ensures that updating _docs_ doesn't cause a workspace rebuild
  workspace = uv2nix.lib.workspace.loadWorkspace { workspaceRoot = ./pyproject; };

  # Create package overlay from workspace.
  overlay = workspace.mkPyprojectOverlay { sourcePreference = "wheel"; };

  # Construct package set, using base package set from pyproject.nix builders
  pythonPkgSet =
    (callPackage pyproject-nix.build.packages { python = python312; }).overrideScope
      overlay;
  pyEnv = pythonPkgSet.mkVirtualEnv "roar-docs-py-env" workspace.deps.default;
  # add Doxygen to the final environment so we can generate code docs
  env = symlinkJoin {
    name = "roar-docs-env";
    paths = [
      pyEnv
      doxygen
    ];
  };
  # create derivation which builds the docs
  docs = stdenv.mkDerivation {
    name = "roar-docs";
    buildInputs = [ env ];
    src = lib.cleanSourceWith {
      name = "roar-docs-src";
      src = lib.cleanSource ./..;
      filter =
        path: type:
        let
          # strip out the absolute path prefix
          relativePath = lib.removePrefix (toString ./. + "/") path;
        in
        !((lib.hasInfix "build/" relativePath) || (lib.hasInfix "generated/" relativePath));
    };

    # make needs to be run from the docs directory, but we need the whole tree for Doxygen generation
    # (hence source being the whole project)
    # note that the _sources directory is removed as we don't want to accidentally leak things
    buildPhase = ''
      cd docs
      make html
      rm -rf ./build/html/_sources
    '';
    # install the docs to $out/html
    installPhase = ''
      mkdir -p $out/html
      cp -a ./build/html/. $out/html
    '';
    # provide dev shell in passthru
    passthru = {
      inherit env compressed decompress;
      shell = mkShell {
        buildInputs = [
          env
          uv # uv added to manage python dependencies
        ];
        shellHook = ''
          # Undo dependency propagation by nixpkgs.
          unset PYTHONPATH
        '';
      };
    };
  };

  compressed = stdenv.mkDerivation {
    name = "roar-docs-compressed";
    buildInputs = [
      docs
      p7zip
    ];
    src = docs;
    installPhase = ''
      mkdir -p $out
      7z a -t7z -m0=lzma -mx=9 -mfb=64 -md=32m -ms=on $out/docs.7z .
    '';
  };
  decompress = writeShellScriptBin "decompress" ''
    ${lib.getExe p7zip} x $1 -o$2
  '';
in
docs
