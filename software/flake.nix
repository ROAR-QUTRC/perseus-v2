{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nix-ros-workspace = {
      url = "github:hacker1024/nix-ros-workspace";
      flake = false;
    };
  };
  outputs =
    {
      nixpkgs,
      nix-ros-overlay,
      nix-ros-workspace,
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
            (import ./ros_ws/colcon/overlay.nix)
            # add ros workspace functionality
            (import nix-ros-workspace { }).overlay
            # import ros workspace packages + fixes
            (import ./overlay.nix rosDistro)
            # finally, alias the output to pkgs.ros to make it easier to use
            (final: prev: {
              ros = final.rosPackages.${rosDistro}.overrideScope (
                rosFinal: rosPrev: { manualDomainId = toString productionDomainId; }
              );
            })
          ];
          # Gazebo makes use of Freeimage.
          # Freeimage is blocked by default since it has a whole bunch of CVEs.
          # This means we have to explicitly permit Freeimage to allow Gazebo to run.
          config.permittedInsecurePackages = [ "freeimage-unstable-2021-11-01" ];
        };

        # --- INPUT PACKAGE SETS ---
        devPackages = pkgs.ros.devPackages // pkgs.sharedDevPackages // pkgs.nativeDevPackages;
        # Packages which should be available in the shell, both in development and production
        standardPkgs = {
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

        # --- OUTPUT NIX WORKSPACES ---
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
              prebuiltShellPackages = devShellPkgs;
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
                    # tell colcon to use our defaults file (uses --symlink-install by default)
                    export COLCON_DEFAULTS_FILE=${./ros_ws/colcon_defaults.yaml}
                  '';
              }
            );
          in
          # override the .env (cli environment) attribute with our modifications
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

        # --- LAUNCH SCRIPTS ---
        perseus-main = pkgs.writeShellScriptBin "perseus-main" ''
          ${default}/bin/ros2 pkg list
        '';
      in
      {
        # rover development environment
        packages = {
          # note: as well as adding the output workspaces,
          # we also output the entire package set to make certain debugging easier
          inherit default simulation pkgs;

          # used only to split up the build for Cachix - this particular package builds only the ROS core,
          # thus reducing RAM required (slightly... still need a fair bit of swap space)
          rosCore = pkgs.ros.callPackage pkgs.ros.buildROSWorkspace { name = "ROS Core"; };
        };

        devShells = {
          default = default.env;
          simulation = simulation.env;
        };

        apps = {
          default = {
            type = "app";
            program = "${pkgs.lib.getExe perseus-main}";
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
