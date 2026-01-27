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
      url = "github:nix-community/home-manager";
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

        productionDomainId = 42;
        devDomainId = 51;

        # --- NIX PACKAGES IMPORT AND OVERLAYS ---
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nixgl.overlay
            # TODO: Look into fixing nix-gl-host? Doesn't work on Intel hardware
            nix-gl-host.overlays.default
            # get the ros packages
            nix-ros-overlay.overlays.default
            # fix colcon (silence warnings, add extensions)
            (import ./software/ros_ws/colcon/overlay.nix)
            # add ros workspace functionality
            nix-ros-workspace.overlays.default
            # import ros workspace packages + fixes
            (import ./software/overlay.nix rosDistro)
            (import ./packages/overlay.nix)
            (final: prev: {
              inherit self; # add self access for hacks like nixGL
              # alias the output to pkgs.ros to make it easier to use
              ros = final.rosPackages.${rosDistro};
              # and add pkgs.unstable access
              unstable = pkgs-unstable;
            })
          ];
          # Gazebo makes use of Freeimage.
          # Freeimage is blocked by default since it has a whole bunch of CVEs.
          # This means we have to explicitly permit Freeimage to allow Gazebo to run.
          config.permittedInsecurePackages = [ "freeimage-unstable-2021-11-01" ];
          config.allowUnfreePredicate =
            pkg:
            builtins.elem (pkgs.lib.getName pkg) [
              "drawio"
              # CUDA packages
              "cuda_cudart"
              "cuda_nvcc"
              "cuda_cccl"
              "libcublas"
              "libcufft"
              "libcurand"
              "libcusolver"
              "libcusparse"
              "libnpp"
              "cuda_nvrtc"
              "cuda_nvml_dev"
              "cuda_profiler_api"
              "cudatoolkit"
              "libnvjitlink"
              "cuda_cuobjdump"
              "cuda_gdb"
              "cuda_cuxxfilt"
              "cuda_nvdisasm"
              "cuda_nvprune"
              "cuda_sanitizer_api"
              "cuda_nvtx"
              # ONNX Runtime CUDA dependencies
              "cudnn"
              "cudnn-frontend"
            ];
        };
        # we don't need to apply overlays here since pkgs-unstable is only for pure python stuff
        pkgs-unstable = import nixpkgs-unstable {
          inherit system;
          overlays = [
            (import ./docs/nix/overlay.nix)
          ];
          config.allowUnfree = true; # needed for draw.io for the docs
        };

        # --- INPUT PACKAGE SETS ---
        devPackages = pkgs.ros.devPackages // pkgs.sharedDevPackages // pkgs.nativeDevPackages;
        # Packages which should be available in the shell, both in development and production
        standardPkgs = {
          inherit (pkgs)
            groot2
            bashInteractive
            can-utils
            corepack_23
            nixgl-script
            nixcuda-script
            ncurses
            glibcLocales
            yaml-cpp
            libnice
            ;
          inherit (pkgs.gst_all_1)
            gstreamer
            gst-plugins-base
            gst-plugins-good
            gst-plugins-bad
            gst-plugins-rs
            ;
          inherit (pkgs.ros)
            twist-stamper
            rosbridge-suite
            livox-ros-driver2
            rviz2-fixed
            rosbag2
            teleop-twist-keyboard
            joy
            demo-nodes-cpp
            tf2-tools
            rqt-gui
            rqt-gui-py
            rqt-graph
            rqt-plot
            rqt-reconfigure
            rqt-common-plugins
            rmw-cyclonedds-cpp
            nav2-rviz-plugins
            opennav-docking
            nav2-msgs
            nav2-util
            nav2-lifecycle-manager
            nav2-common
            ;
        };
        # Packages which should be available only in the dev shell
        devShellPkgs = {
          inherit (pkgs)
            man-pages
            man-pages-posix
            stdmanpages
            nix-gl-host
            ;
          inherit (pkgs.cudaPackages)
            cuda_nvcc
            cuda_cudart
            cuda_cccl
            libcublas
            libcufft
            libcurand
            libcusolver
            libcusparse
            cuda_nvrtc
            cudnn
            ;
          # ONNX Runtime with CUDA support
          onnxruntime-cuda = pkgs.onnxruntime.override { cudaSupport = true; };
        };
        # Packages needed to run the simulation
        # Note: May not be needed, most needed packages should
        # already be brought in as deps of the simulation workspace packages
        simPkgs = { };

        # --- ROS WORKSPACES ---
        # function to build a ROS workspace which modifies the dev shell hook to set up environment variables
        mkWorkspace =
          {
            ros,
            name ? "ROAR",
            additionalDevPkgs ? { },
            additionalPkgs ? { },
          }:
          ros.callPackage ros.buildROSWorkspace {
            inherit name;
            devPackages = devPackages // additionalDevPkgs;
            prebuiltPackages = standardPkgs // additionalPkgs;
            prebuiltShellPackages = devShellPkgs // formatters;
            releaseDomainId = productionDomainId;
            environmentDomainId = devDomainId;
            forceReleaseDomainId = true;

            postShellHook = ''
              # use CycloneDDS ROS middleware
              export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
              # enable coloured ros2 launch output
              export RCUTILS_COLORIZED_OUTPUT=1
              # fix locale issues
              export LOCALE_ARCHIVE=${pkgs.glibcLocales}/lib/locale/locale-archive
              # CUDA environment setup
              export CUDA_PATH="${pkgs.cudaPackages.cuda_nvcc}"
              export CUDA_HOME="${pkgs.cudaPackages.cuda_nvcc}"
              export NVCC_PREPEND_FLAGS="-ccbin ${pkgs.gcc}/bin"
              # ONNX Runtime with CUDA support
              export ORT_LIB_LOCATION="${devShellPkgs.onnxruntime-cuda}/lib"
            '';
          };

        # Actually build the workspaces
        default = mkWorkspace {
          inherit (pkgs) ros;
          name = "ROAR";
        };
        simulation = mkWorkspace {
          inherit (pkgs) ros;
          name = "ROAR Simulation";
          additionalPkgs = simPkgs;
          additionalDevPkgs = pkgs.ros.simDevPackages;
        };

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
        };

        devShells = {
          default = default.env;
          simulation = simulation.env;

          docs = docs.shell;
        };

        apps =
          let
            mkRosLaunchScript =
              name: package: launchFile:
              pkgs.writeShellScriptBin name ''
                ${default}/bin/ros2 launch ${package} ${launchFile} "$@"
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
