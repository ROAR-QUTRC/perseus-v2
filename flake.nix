{
  inputs = {
    # ros inputs
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nix-ros-workspace = {
      url = "github:RandomSpaceship/nix-ros-workspace";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.nix-ros-overlay.follows = "nix-ros-overlay";
    };
    # docs inputs
    nixpkgs-unstable.url = "nixpkgs/nixpkgs-unstable";
    pyproject-nix = {
      url = "github:pyproject-nix/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs-unstable";
    };
    uv2nix = {
      url = "github:adisbladis/uv2nix";
      inputs.nixpkgs.follows = "nixpkgs-unstable";
      inputs.pyproject-nix.follows = "pyproject-nix";
    };
    pyproject-build-systems = {
      url = "github:pyproject-nix/build-system-pkgs";
      inputs.pyproject-nix.follows = "pyproject-nix";
      inputs.uv2nix.follows = "uv2nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # flake compat input (allows use of nix-build and nix-shell for the default workspace)
    flake-compat.url = "https://flakehub.com/f/edolstra/flake-compat/1.tar.gz";
    # formatting
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # home-manager (for device setup)
    home-manager = {
      url = "github:nix-community/home-manager/release-25.11";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # note: nix-gl-host works way better than NixGL on Nvidia hardware,
    # but breaks down everywhere else. Might look into fixing it at some point,
    # since it's both faster and in my opinion a better solution than NixGL.
    nix-gl-host = {
      url = "github:numtide/nix-gl-host";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixgl = {
      url = "github:nix-community/nixGL";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-utils.follows = "nix-ros-overlay/flake-utils";
      };
    };
  };
  outputs =
    {
      self,
      nixpkgs,
      nix-ros-overlay,
      nix-ros-workspace,
      nixpkgs-unstable,
      pyproject-nix,
      uv2nix,
      pyproject-build-systems,
      treefmt-nix,
      nix-gl-host,
      nixgl,
      ...
    }@inputs:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        # --- INPUT PARAMETERS ---
        rosDistro = "jazzy";

        # --- NIX PACKAGES IMPORT AND OVERLAYS ---
        pkgs = import nixpkgs {
          inherit system;
          # These are imports from ./nix to reduce the amount of clutter in the flake
          overlays = import ./nix/nixpkgs-overlays.nix {
            inherit
              inputs
              pkgs-unstable
              rosDistro
              self
              ;
          };
          config = import ./nix/nixpkgs-config.nix;
        };
        # we don't need to apply overlays here since pkgs-unstable is only for pure python stuff
        pkgs-unstable = import nixpkgs-unstable {
          inherit system;
          config.allowUnfree = true; # needed for draw.io for the docs
        };

        # --- FORMATTING ---
        treefmtEval = treefmt-nix.lib.evalModule pkgs-unstable ./treefmt.nix;

        # --- ROS Workspaces ---
        # note: treefmtEval is used as the formatter in the ROS workspaces
        # The current workspaces are: default, simulation, and machineLearning
        rosWorkspaces = import ./nix/workspaces.nix { inherit pkgs treefmtEval; };

        # --- PYTHON (UV) WORKSPACES ---
        # note: called with rosDistro to link correct intersphinx inventory
        docs = pkgs.callPackage (import ./docs) {
          inherit
            rosDistro
            pyproject-nix
            uv2nix
            pyproject-build-systems
            ;
        };

        firmware = (
          pkgs.buildFHSEnv (
            pkgs.appimageTools.defaultFhsEnvArgs
            // {
              name = "ROAR Firmware";
              targetPkgs =
                pkgs:
                (with pkgs; [
                  platformio-core
                ]);
              # runScript = "env LD_LIBRARY_PATH= bash";
            }
          )
        );

        # --- SCRIPTS ---
        treefmt-write-config = pkgs.writeShellScriptBin "treefmt-write-config" ''
          cd "$(git rev-parse --show-toplevel)"
          cp ${treefmtEval.config.build.configFile} ./treefmt.toml
          chmod +w treefmt.toml
          # strip out Nix store prefix path from the config,
          # along with ruff-check (it just causes errors when trying to "format" in VSCode,
          # since it's a linter)
          sed -i -e 's,command.*/,command = ",' -e "/\[formatter\.ruff-check\]/,/^$/d" treefmt.toml
        '';

      in
      {
        # rover development environment
        packages = {
          inherit docs firmware;

          # Output the entire package set to make certain debugging easier
          # Note that it needs to be a derivation though to make nix flake commands happy, so we just touch the output file
          # so that it can "build" successfully
          pkgs = pkgs.runCommandNoCC "roar-all-pkgs" { passthru = pkgs; } ''
            touch $out
          '';

          # same as pkgs but for utilities
          tools =
            pkgs.runCommandNoCC "roar-tools"
              {
                passthru = {
                  inherit treefmt-write-config;
                  treefmt-build = treefmtEval.config.build;
                };
              }
              ''
                mkdir $out
              '';
          scripts =
            pkgs.runCommandNoCC "roar-scripts"
              {
                passthru = pkgs.scripts;
              }
              ''
                mkdir $out
              '';
        }
        # Include the ROS workspaces from ./nix/workspaces.nix
        // rosWorkspaces;

        devShells = {
          default = rosWorkspaces.default.env;
          simulation = rosWorkspaces.simulation.env;
          machineLearning = rosWorkspaces.machineLearning.env;
          firmware = firmware.env;
          docs = docs.shell;
        };

        apps =
          let
            mkRosLaunchScript =
              name: package: launchFile:
              pkgs.writeShellScriptBin name ''
                ${rosWorkspaces.default}/bin/ros2 launch ${package} ${launchFile} "$@"
              '';
            mkRosLaunchApp =
              name: package: launchFile:
              let
                script = mkRosLaunchScript name package launchFile;
              in
              {
                type = "app";
                program = "${pkgs.lib.getExe script}";
              };
          in
          {
            perseus = mkRosLaunchApp "perseus" "perseus" "perseus.launch.py";
            perseus-lite = mkRosLaunchApp "perseus-lite" "perseus_lite" "perseus_lite.launch.py";
            default = self.apps.${system}.perseus;
            generic_controller = mkRosLaunchApp "generic_controller" "perseus_input" "controller.launch.py";
            ros2 = {
              type = "app";
              program = "${rosWorkspaces.default}/bin/ros2";
            };
            clean = {
              type = "app";
              program = "${pkgs.scripts.clean}/bin/clean";
            };
          };
        formatter = treefmtEval.config.build.wrapper;
      }
    )
    // {
      homeConfigurations = (import ./software/home-manager/default.nix) inputs;
    };
  nixConfig = {
    extra-substituters = [
      "https://roar-qutrc.cachix.org"
      "https://ros.cachix.org"
    ];

    # Note that this is normally a VERY BAD IDEA but is needed to make building the docs easier.
    # Several steps in the build process require either internet access
    # or annoying workarounds, and internet access removes a maintenance burden.
    # sandbox = "relaxed";
    # This setting is set during the Github Actions to allow the online docs to be built with internet access
    # without having to have all members set themselves as 'trusted-users' in nix.conf
  };
}
