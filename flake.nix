{
  inputs = {
    # ros inputs
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nix-ros-workspace = {
      url = "github:RandomSpaceship/nix-ros-workspace";
      flake = false;
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
      ...
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        # --- INPUT PARAMETERS ---
        rosDistro = "humble";

        productionDomainId = 42;
        devDomainId = 51;

        # --- NIX PACKAGES IMPORT AND OVERLAYS ---
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            # get the ros packages
            nix-ros-overlay.overlays.default
            # fix colcon (silence warnings, add extensions)
            (import ./software/ros_ws/colcon/overlay.nix)
            # add ros workspace functionality
            (import nix-ros-workspace { }).overlay
            # import ros workspace packages + fixes
            (import ./software/overlay.nix rosDistro)
            (final: prev: {
              # alias the output to pkgs.ros to make it easier to use
              ros = final.rosPackages.${rosDistro}.overrideScope (
                rosFinal: rosPrev: { manualDomainId = toString productionDomainId; }
              );
              # override select packages with latest
              inherit (pkgs-unstable) uv;
            })
          ];
          # Gazebo makes use of Freeimage.
          # Freeimage is blocked by default since it has a whole bunch of CVEs.
          # This means we have to explicitly permit Freeimage to allow Gazebo to run.
          config.permittedInsecurePackages = [ "freeimage-unstable-2021-11-01" ];
        };
        # we don't need to apply overlays here since pkgs-unstable is only for pure python stuff
        pkgs-unstable = import nixpkgs-unstable {
          inherit system;
          config.allowUnfree = true; # needed for draw.io for the docs
        };

        # --- INPUT PACKAGE SETS ---
        devPackages = pkgs.ros.devPackages // pkgs.sharedDevPackages // pkgs.nativeDevPackages;
        # Packages which should be available in the shell, both in development and production
        standardPkgs = {
          inherit (pkgs) can-utils;
          inherit (pkgs.ros)
            natrviz
            rviz2
            rosbag2
            teleop-twist-keyboard
            demo-nodes-cpp
            ;
        };
        # Packages which should be available only in the dev shell
        devShellPkgs = {
          inherit (pkgs) man-pages man-pages-posix stdmanpages;
        };
        # Packages needed to run the simulation
        simPkgs = {
          inherit (pkgs.ros) gazebo-ros gazebo-ros2-control gazebo-ros-pkgs;
        };

        # --- ROS WORKSPACES ---
        # function to build a ROS workspace which modifies the dev shell hook to set up environment variables
        mkWorkspace =
          {
            ros,
            name ? "ROAR",
            additionalPkgs ? { },
          }:
          let
            workspace = ros.callPackage ros.buildROSWorkspace {
              inherit devPackages name;
              prebuiltPackages = standardPkgs // additionalPkgs;
              prebuiltShellPackages = devShellPkgs // formatters;
            };
            env = workspace.env.overrideAttrs (
              {
                shellHook ? "",
                ...
              }:
              {
                # override the shell hook to set some environment variables
                shellHook =
                  shellHook
                  + ''
                    # set the ROS_DOMAIN_ID to the development ID, since by default it's set to the production ID
                    export ROS_DOMAIN_ID=${toString devDomainId}
                  '';
              }
            );
          in
          # override the env attribute (cli environment) with our modifications
          workspace // { inherit env; };

        # Actually build the workspaces
        default = mkWorkspace {
          inherit (pkgs) ros;
          name = "ROAR";
        };
        simulation = mkWorkspace {
          inherit (pkgs) ros;
          name = "ROAR Simulation";
          additionalPkgs = simPkgs;
        };

        # --- PYTHON (UV) WORKSPACES ---
        # note: called with pkgs-unstable since we need the uv tool to be up-to-date due to rapid development
        # note: called with rosDistro to link correct intersphinx inventory
        docs = pkgs.callPackage (import ./docs) {
          inherit
            rosDistro
            pyproject-nix
            uv2nix
            pyproject-build-systems
            ;
        };

        # --- SCRIPTS ---
        perseus = pkgs.writeShellScriptBin "perseus" ''
          ${default}/bin/ros2 pkg list
        '';
        treefmt-write-config = pkgs.writeShellScriptBin "treefmt-write-config" ''
          cd "$(git rev-parse --show-toplevel)"
          cp ${treefmtEval.config.build.configFile} ./treefmt.toml
          chmod +w treefmt.toml
        '';

        # --- FORMATTING ---
        treefmtEval = treefmt-nix.lib.evalModule pkgs-unstable ./treefmt.nix;

        # formatters package set for use in ROS workspaces
        formatters =
          {
            # include treefmt wrapped with the config from ./treefmt.nix
            treefmt = treefmtEval.config.build.wrapper;
          }
          # plus all of the individual formatter programs from said config
          // treefmtEval.config.build.programs;
      in
      {
        # rover development environment
        packages = {
          inherit default simulation docs;

          # Output the entire package set to make certain debugging easier 
          # Note that it needs to be a derivation though to make nix flake commands happy, so we just touch the output file
          # so that it can "build" successfully
          pkgs = pkgs.runCommand "roar-all-pkgs" { passthru = pkgs; } ''
            touch $out
          '';

          # same as pkgs but for utilities
          tools =
            pkgs.runCommand "roar-tools"
              {
                passthru = {
                  inherit (pkgs) scripts;
                  inherit treefmt-write-config;
                  treefmt-build = treefmtEval.config.build;
                };
              }
              ''
                touch $out
              '';
        };

        devShells = {
          default = default.env;
          simulation = simulation.env;

          docs = docs.shell;
        };

        apps = {
          default = {
            type = "app";
            program = "${pkgs.lib.getExe perseus}";
          };
          ros2 = {
            type = "app";
            program = "${default}/bin/ros2";
          };
          clean = {
            type = "app";
            program = "${pkgs.scripts.clean}/bin/clean";
          };
        };
        formatter = treefmtEval.config.build.wrapper;
        checks = {
          # formatting = treefmtEval.config.build.check self;
        };
      }
    );
  nixConfig = {
    # note from James Nichol - I set up a custom cache at https://qutrc-roar.cachix.org 
    # Currently I'm compiling for x86-64 and aarch64 on my machine and pushing to it whenever I make changes
    # to the Nix config - contact me if you want an auth token to push your own builds
    extra-substituters = [
      "https://qutrc-roar.cachix.org"
      "https://ros.cachix.org"
    ];
    extra-trusted-public-keys = [
      "qutrc-roar.cachix.org-1:lARPhJL+PLuGd021HeN8CQOGGiYVEVGws5za+39M1Z0="
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
    # note that this is normally a VERY BAD IDEA but it may be needed so the docs can have internet access,
    # with certain configurations. Currently, everything is configured to work offline with cached files in the git repo.

    # Unless otherwise configured, sphinx-immaterial will pull fonts from Google's CDN, which obviously requires internet access.
    # The Roboto (and RobotoMono) fonts have been downloaded locally and are used instead.
    # These should never change, so it really doesn't matter.

    # intersphinx also normally expects to be able to download inventory (.inv) files from the target projects,
    # but it has also been configured to use local copies. Updating these files is done with `nix run .#docs.fetch-inventories`,
    # and is run automatically from the CI pipeline (it makes a commit with the update).

    # sandbox = "relaxed";
  };
}
