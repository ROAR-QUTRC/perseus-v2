{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nix-ros-workspace = {
      url = "github:hacker1024/nix-ros-workspace";
      flake = false;
    };
  };
  outputs = { self, nix-ros-overlay, nix-ros-workspace, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        # --- BOILERPLATE & CONFIG ---
        rosDistro = "humble";

        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            (import nix-ros-workspace { }).overlay
            (final: prev:
              let ros = prev.rosPackages.${rosDistro};
              in {
                inherit ros;
                buildColconPackage = ros.callPackage ./build-colcon-package {
                  inherit rosDistro;
                  rosVersion = 2;
                };
              })
            (import ./overlays.nix)
            (final: prev: {
              ros = prev.ros // prev.ros.overrideScope
                (import ./nix-ros-packages/overlay.nix);
            })
          ];
          # Gazebo makes use of Freeimage.
          # Freeimage is blocked by default since it has a whole bunch of CVEs.
          # This means we have to explicitly permit Freeimage to allow Gazebo to run.
          config.permittedInsecurePackages =
            [ "freeimage-unstable-2021-11-01" ];
        };

        # --- ROVER PACKAGES ---
        dev-packages = {
          # mini package which puts COLCON_IGNORE in the output result folder
          # allows colcon build of workspace after run nix build
          colcon-ignore = pkgs.stdenv.mkDerivation rec {
            inherit system;
            dontUnpack = true;
            name = "colcon-ignore";
            installPhase = ''
              mkdir -p $out
              touch $out/COLCON_IGNORE
            '';
          };
        } // (builtins.intersectAttrs
          (import ./nix-ros-packages/overlay.nix null null) pkgs.ros);

        # packages needed to build the workspace
        build-pkgs = {
          inherit (pkgs) colcon python311;
          inherit (pkgs.ros) ament-cmake-core python-cmake-module;
        };
        # CLI tools
        cli-tool-pkgs = {
          inherit (pkgs.ros) rviz2 rosbag2 teleop-twist-keyboard demo-nodes-cpp;
        };

        # core packages needed to run everything
        core-pkgs = {
          inherit (pkgs.ros) ros-core;
          inherit (pkgs.python311Packages) pygame;
        };

        # --- SUBSYSTEM PACKAGE SETS ---
        # Almost all of these should actually disappear as the workspace gets progressively nix-ified,
        # since the individual packages will define their own dependencies
        vision-pkgs = { };
        arm-pkgs = { };

        # packages for simulation
        sim-pkgs = {
          inherit (pkgs.ros)
            gazebo-ros gazebo-ros2-control gazebo-ros-pkgs controller-manager
            topic-tools robot-localization slam-toolbox imu-filter-madgwick
            laser-filters joint-state-publisher-gui joint-state-broadcaster;
        };

        # final merged set of packages for both sim and rover core development
        common-pkgs = core-pkgs // cli-tool-pkgs // vision-pkgs // arm-pkgs;

        # --- OUTPUT NIX WORKSPACES ---
        default = pkgs.ros.callPackage pkgs.ros.buildROSWorkspace {
          interactive = true;
          name = "ROAR";
          devPackages = dev-packages;
          prebuiltPackages = common-pkgs;
          prebuiltShellPackages = build-pkgs;
        };

        # rover simulation environment with Gazebo, etc
        roverSim = pkgs.ros.callPackage pkgs.ros.buildROSWorkspace {
          name = "ROAR Simulation";
          devPackages = dev-packages;
          prebuiltPackages = common-pkgs // sim-pkgs;
          prebuiltShellPackages = { } // build-pkgs;
        };

        # LAUNCH SCRIPTS
        perseus-main = pkgs.writeShellScriptBin "perseus-main" ''
          ${default}/bin/ros2 pkg list
        '';
      in {
        # rover development environment
        packages = {
          inherit default roverSim;
          # used only to split up the build for Cachix - this particular package builds only the ROS core,
          # thus reducing RAM required
          rosCore = pkgs.ros.callPackage pkgs.ros.buildROSWorkspace {
            name = "ROS Core";
          };
        };

        devShells = {
          default = default.env;
          roverSim = roverSim.env;
        };
        apps = {
          default = {
            type = "app";
            program = "${perseus-main}/bin/perseus-main";
          };
        };
      });
  nixConfig = {
    # note from James Nichol - I set up a custom cache at https://qutrc-roar.cachix.org 
    # Currently I'm compiling for x86-64 and aarch64 on my machine and pushing to it whenever I make changes
    # to the Nix config - contact me if you want an auth token to push your own builds
    extra-substituters =
      [ "https://qutrc-roar.cachix.org" "https://ros.cachix.org" ];
    extra-trusted-public-keys = [
      "qutrc-roar.cachix.org-1:lARPhJL+PLuGd021HeN8CQOGGiYVEVGws5za+39M1Z0="
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };
}
