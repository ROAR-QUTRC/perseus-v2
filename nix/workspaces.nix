{ pkgs, treefmtEval, ... }:
let
  inherit (pkgs.stdenv.hostPlatform) isx86_64;

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
      rtabmap-odom
      rtabmap-slam
      rtabmap-util
      rtabmap-rviz-plugins
      rtabmap-viz
      ;
  }
  # Open3D is x86_64-linux only (uses Intel MKL/IPP)
  // pkgs.lib.optionalAttrs isx86_64 { inherit (pkgs) open3d; };
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

  # Python environment containing Open3D's Python runtime dependencies (plotly, dash, etc.)
  # Used to construct PYTHONPATH in the shell hook so `import open3d` works (x86_64 only)
  open3dPythonDeps = pkgs.lib.optionalAttrs isx86_64 {
    env = pkgs.python3.withPackages pkgs.open3d.pythonDeps;
  };

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
        ${pkgs.lib.optionalString isx86_64 ''
          # Open3D Python module and its Python dependencies (plotly, dash, etc.)
          export PYTHONPATH="${pkgs.open3d}/lib/python${pkgs.python3.pythonVersion}/site-packages:${open3dPythonDeps.env}/lib/python${pkgs.python3.pythonVersion}/site-packages''${PYTHONPATH:+:$PYTHONPATH}"
        ''}
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
    additionalDevPkgs = pkgs.ros.simDevPackages;
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
