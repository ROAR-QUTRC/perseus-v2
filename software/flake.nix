{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nix-ros-workspace = {
      url = "github:hacker1024/nix-ros-workspace";
      flake = false;
    };
    nixGL = {
      url = "github:nix-community/nixGL";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs =
    {
      self,
      nixpkgs,
      nix-ros-overlay,
      nix-ros-workspace,
      nixGL,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        # --- NIX PACKAGES IMPORT AND OVERLAYS ---
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            # get the ros packages
            nix-ros-overlay.overlays.default
            # add ros workspace functionality
            (import nix-ros-workspace { }).overlay
            # import ros workspace packages + fixes
            (import ./overlay.nix)
          ];
          # Gazebo makes use of Freeimage.
          # Freeimage is blocked by default since it has a whole bunch of CVEs.
          # This means we have to explicitly permit Freeimage to allow Gazebo to run.
          config.permittedInsecurePackages = [ "freeimage-unstable-2021-11-01" ];
        };

        devPackages = pkgs.rosDevPackages // pkgs.sharedDevPackages // pkgs.nativeDevPackages;
        # Packages which should be available in the shell, both in development and production
        standardPkgs = {
          inherit (pkgs.ros)
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

        # --- OUTPUT NIX WORKSPACES ---
        default = pkgs.ros.callPackage pkgs.ros.buildROSWorkspace {
          inherit devPackages;
          name = "ROAR";
          prebuiltPackages = standardPkgs;
          prebuiltShellPackages = devShellPkgs;
        };

        # rover simulation environment with Gazebo, etc
        roverSim = pkgs.ros.callPackage pkgs.ros.buildROSWorkspace {
          inherit devPackages;
          name = "ROAR Simulation";
          prebuiltPackages = standardPkgs // {
            inherit (pkgs.ros) gazebo-ros gazebo-ros2-control gazebo-ros-pkgs;
          };
          prebuiltShellPackages = devShellPkgs;
        };

        # --- LAUNCH SCRIPTS ---
        perseus-main = pkgs.writeShellScriptBin "perseus-main" ''
          ${default}/bin/ros2 pkg list
        '';
        rviz2-nixgl = pkgs.writeShellScriptBin "rviz2-nixgl" ''
          NIXPKGS_ALLOW_UNFREE=1 QT_QPA_PLATFORM=xcb QT_SCREEN_SCALE_FACTORS=1 nix run --impure "github:nix-community/nixGL" "${default}/bin/rviz2" -- "$@"
        '';
      in
      {
        # rover development environment
        packages = {
          inherit default roverSim;
          inherit (pkgs) ros;
          # used only to split up the build for Cachix - this particular package builds only the ROS core,
          # thus reducing RAM required (slightly... still need a fair bit of swap space)
          rosCore = pkgs.ros.callPackage pkgs.ros.buildROSWorkspace { name = "ROS Core"; };
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
          rviz2 = {
            type = "app";
            program = "${rviz2-nixgl}/bin/rviz2-nixgl";
          };
        };
        formatter = pkgs.nixfmt-rfc-style;
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
  };
}
