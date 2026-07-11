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
  stdenv,
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

  # Upstream hardcodes -std=c++0x (C++11), but Sophus 1.24.6 headers require
  # C++14/17 (std::enable_if_t). Bump to C++17 to match vikit_ros.
  #
  # Also strip `-march=native`: it bakes the build machine's ISA into the
  # artifact, which breaks reproducibility and risks SIGILL when a binary-cached
  # build (e.g. from CI) is run on robot hardware with a different CPU.
  postPatch = ''
    substituteInPlace CMakeLists.txt \
      --replace-warn "-std=c++0x" "-std=c++17" \
      --replace-warn "-std=c++11" "-std=c++17" \
      --replace-warn "-march=native " ""
  ''
  # Upstream gates ARM detection on the ARM_ARCHITECTURE env var, which is never
  # set inside the Nix sandbox, so it applies x86 SSE flags (-msse*, -mmmx) even
  # on aarch64, where g++ rejects them. Strip those flags on non-x86 platforms.
  + lib.optionalString (!stdenv.hostPlatform.isx86_64) ''
    substituteInPlace CMakeLists.txt \
      --replace-warn "-mmmx -msse -msse -msse2 -msse3 -mssse3" ""
  '';

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
    # Upstream package.xml declares "GPLv3" with no "or later" clause -> GPL-3.0-only.
    license = with lib.licenses; [ gpl3Only ];
  };
}
