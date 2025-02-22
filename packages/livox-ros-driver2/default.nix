{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  pkg-config,
  ros,
  livox-sdk2,
  pcl,
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
    ros.ament-cmake-auto
    ros.ament-cmake-ros
    ros.rosidl-default-generators
  ];

  buildInputs = [
    ros.rclcpp
    ros.sensor-msgs
    ros.std-msgs
    ros.tf2
    ros.tf2-ros
    livox-sdk2
    ros.ament-index-cpp
    ros.rosidl-default-runtime
    ros.rclcpp-components
    pcl
  ];

  prePatch = ''
    # Setup package.xml
    cp package_ROS2.xml package.xml
    sed -i '/<\/package>/i \  <member_of_group>rosidl_interface_packages</member_of_group>' package.xml

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
    export CPATH="$CPATH:${ros.rclcpp}/include:${ros.sensor-msgs}/include:${livox-sdk2}/include:${ros.rclcpp-components}/include"
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${livox-sdk2}/lib"
  '';

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
