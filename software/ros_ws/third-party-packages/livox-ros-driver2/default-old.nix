{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  pkg-config,
  livox-sdk2,
  pcl,
  amen,
}:

stdenv.mkDerivation rec {
  pname = "livox-ros-driver2";
  version = "1.2.4";

  src = fetchFromGitHub {
    owner = "Livox-SDK";
    repo = "livox_ros_driver2";
    rev = version;
    hash = "sha256-6Z2mvpdKRYD/hoL+laAcBYSzPNDqof5b1RjrPJaHIUQ=";
  };

  nativeBuildInputs = [
    cmake
    pkg-config
    ament-cmake-auto
    ament-cmake-ros
    rosidl-default-generators
  ];

  buildInputs = [
    rclcpp
    sensor-msgs
    std-msgs
    tf2
    tf2-ros
    livox-sdk2
    ament-index-cpp
    rosidl-default-runtime
    rclcpp-components
    pcl
  ];

  prePatch = ''
    # Modify CMakeLists.txt to set ROS_EDITION to ROS2
    sed -i 's/if(ROS_EDITION STREQUAL "ROS2")/if(TRUE)/' CMakeLists.txt
  '';

  cmakeFlags = [
    "-DROS_VERSION=2"
    "-DROS_DISTRO=jazzy"
    "-DHUMBLE_ROS=OFF"
    "-DCMAKE_INSTALL_PREFIX=${placeholder "out"}"
    "-DCMAKE_BUILD_TYPE=Release"
    "-DBUILD_SHARED_LIBS=ON"
    "-DCMAKE_MODULE_PATH=${placeholder "out"}/share/cmake"
    "-DCMAKE_PREFIX_PATH=${livox-sdk2};${pcl}"
  ];

  preConfigure = ''
    # Ensure cmake modules directory exists
    mkdir -p cmake/modules

    # Copy cmake modules if they exist
    if [ -d ${src}/cmake/modules ]; then
      cp -r ${src}/cmake/modules/* cmake/modules/
    fi

    # Copy version.cmake 
    if [ -f ${src}/cmake/version.cmake ]; then
      cp ${src}/cmake/version.cmake cmake/
    fi

    export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH''${AMENT_PREFIX_PATH:+:}$out/share"
    export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:${livox-sdk2}:${pcl}"
    export CPATH="$CPATH:${rclcpp}/include:${sensor-msgs}/include:${livox-sdk2}/include:${rclcpp-components}/include"
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${livox-sdk2}/lib"
  '';

  patches = [

  ];

  postInstall = ''
    # Install include files
    mkdir -p $out/include/livox_ros_driver2
    cp -r include/* $out/include/livox_ros_driver2/

    # Install message files
    mkdir -p $out/share/${pname}/msg
    cp -r msg/*.msg $out/share/${pname}/msg/
  '';

  meta = with lib; {
    description = "Livox ROS Driver 2.0 for Livox LiDAR";
    homepage = "https://github.com/Livox-SDK/livox_ros_driver2";
    license = licenses.mit;
    platforms = platforms.linux;
  };
}
