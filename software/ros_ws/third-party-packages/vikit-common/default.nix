# Vikit (Vision-Kit for Robotics) - common library, ROS2 Jazzy port.
# Source is the rpg_vikit monorepo; this package is the vikit_common subdirectory.
#
# vikit_common is a *pure CMake* project (its CMakeLists has USE_ROS=False) that
# does not install a CMake config package - it only registers one in the user's
# ~/.cmake package registry via export(PACKAGE), which does not work under Nix.
# Downstream packages (vikit_ros, fast_livo) do `find_package(vikit_common)`, so
# we install a relocatable vikit_commonConfig.cmake into $out pointing at the
# installed headers and library.
{
  lib,
  buildRosPackage,
  fetchFromGitHub,
  ament-cmake,
  rclcpp,
  # non-ROS system dependencies (resolved from the top-level package set)
  eigen,
  opencv,
  sophus,
  fmt,
}:
let
  src = fetchFromGitHub {
    owner = "Robotic-Developer-Road";
    repo = "rpg_vikit";
    rev = "4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50";
    sha256 = "03sgaghr14fxm0yh2z8837j9722z6nmpp09srifkn12kilbc7hi6";
  };
in
buildRosPackage {
  pname = "ros-jazzy-vikit-common";
  version = "0.0.0";

  inherit src;
  # rpg_vikit is a monorepo; build only the vikit_common package
  sourceRoot = "${src.name}/vikit_common";

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    rclcpp
    eigen
    opencv
    sophus
    fmt
  ];
  nativeBuildInputs = [ ament-cmake ];

  # Install a relocatable CMake config so downstream find_package(vikit_common) works.
  postInstall = ''
    mkdir -p "$out/lib/cmake/vikit_common"
    cat > "$out/lib/cmake/vikit_common/vikit_commonConfig.cmake" <<EOF
    set(vikit_common_INCLUDE_DIR  "$out/include")
    set(vikit_common_INCLUDE_DIRS "$out/include")
    set(vikit_common_LIBRARIES    "$out/lib/libvikit_common.so")
    set(vikit_common_LIBRARY      "$out/lib/libvikit_common.so")
    set(vikit_common_LIBRARY_DIR  "$out/lib")
    set(vikit_common_LIBRARY_DIRS "$out/lib")
    EOF
  '';

  meta = {
    description = "Vikit common: camera models, math and interpolation utilities (ROS2 port)";
    license = with lib.licenses; [ gpl3Plus ];
  };
}
