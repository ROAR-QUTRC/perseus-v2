{ pkgs, treefmtEval, ... }:
let
  productionDomainId = 42;
  devDomainId = 51;

  # --- INPUT PACKAGE SETS ---
  devPackages = pkgs.ros.devPackages // pkgs.sharedDevPackages // pkgs.nativeDevPackages;
  # Packages which should be available in the shell, both in development and production
  standardPkgs = {
    inherit (pkgs)
      groot2
      bashInteractive
      can-utils
      corepack_24
      nodejs_24
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
  };
  # Packages needed to run the simulation
  # Note: May not be needed, most needed packages should
  # already be brought in as deps of the simulation workspace packages
  simPkgs = { };

  cudaPkgs = {
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
  # formatters package set for use in ROS workspaces
  formatters = {
    # include treefmt wrapped with the config from ../treefmt.nix
    treefmt = treefmtEval.config.build.wrapper;
  }
  # plus all of the individual formatter programs from said config
  // treefmtEval.config.build.programs;

  # --- ROS WORKSPACES ---
  # function to build a ROS workspace which modifies the dev shell hook to set up environment variables
  mkWorkspace =
    {
      ros,
      name ? "ROAR",
      additionalDevPkgs ? { },
      additionalPkgs ? { },
      additionalPrebuiltPkgs ? { },
      additionalPostShellHook ? "",
    }:
    ros.callPackage ros.buildROSWorkspace {
      inherit name;
      devPackages = devPackages // additionalDevPkgs;
      prebuiltPackages = standardPkgs // additionalPkgs;
      prebuiltShellPackages = devShellPkgs // formatters // additionalPrebuiltPkgs;
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
      ''
      + additionalPostShellHook;
    };

in
{
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
  machineLearning = mkWorkspace {
    inherit (pkgs) ros;
    name = "ROAR Machine Learning";
    additionalPrebuiltPkgs = cudaPkgs;
    additionalPostShellHook = ''
      # CUDA environment setup
      export CUDA_PATH="${pkgs.cudaPackages.cuda_nvcc}"
      export CUDA_HOME="${pkgs.cudaPackages.cuda_nvcc}"
      export NVCC_PREPEND_FLAGS="-ccbin ${pkgs.gcc}/bin"
      # ONNX Runtime with CUDA support
      export ORT_LIB_LOCATION="${cudaPkgs.onnxruntime-cuda}/lib"
    '';
  };
}
