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
    pcl
  ];

  # Define the LIVOX_INTERFACES_INCLUDE_DIRECTORIES variable
  LIVOX_INTERFACES_INCLUDE_DIRECTORIES = "${livox-sdk2.dev}/include"; # Added

  # Patch phase to set up package.xml and ensure msg directory is intact
  prePatch = ''
    cp package_ROS2.xml package.xml
    sed -i '/<\/package>/i \  <member_of_group>rosidl_interface_packages</member_of_group>' package.xml
    mkdir -p msg
    cp -f ${src}/msg/*.msg msg/
    ls -l msg/
    # Patch CMakeLists.txt to delay include with hardcoded project name and escaped variable
    # Corrected sed command: remove /d and fix pattern matching
    sed -i '/target_include_directories(livox_ros_driver2 PUBLIC/a target_include_directories(livox_ros_driver2 PUBLIC \${LIVOX_INTERFACES_INCLUDE_DIRECTORIES})' CMakeLists.txt
    # Remove the second sed command as it's redundant with the corrected first one
  '';

  cmakeFlags = [
    "-DROS_VERSION=2"
    "-DROS_DISTRO=jazzy"
    "-DCMAKE_INSTALL_PREFIX=${placeholder "out"}/share/${pname}-${version}"
    "--trace-expand" # For detailed CMake debugging
    # Pass the LIVOX_INTERFACES_INCLUDE_DIRECTORIES to CMake
    "-DLIVOX_INTERFACES_INCLUDE_DIRECTORIES=${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}" # Added
  ];

  # Set environment variables
  AMENT_PREFIX_PATH = lib.makeSearchPath "share" ([
    ros.rclcpp
    ros.sensor-msgs
    ros.std-msgs
    ros.tf2
    ros.tf2-ros
    ros.rosidl-default-generators
    livox-sdk2
  ]);

  PCL_DIR = "${pcl}/share/pcl-1.13";

  # Configure build environment
  preConfigure = ''
    export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH''${AMENT_PREFIX_PATH:+:}$out/share"
    export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:${livox-sdk2}:${pcl}"
    export CPATH="$CPATH:${ros.rclcpp}/include:${ros.sensor-msgs}/include:${livox-sdk2.dev}/include" # Updated to use .dev for includes

    # Verify msg directory contents
    if ls msg/*.msg >/dev/null 2>&1; then
      echo "Message files found in msg directory:"
      ls -l msg/
    else
      echo "ERROR: No .msg files found in msg directory. Build will fail."
      exit 1
    fi
  '';

  # Install files to correct locations
  postInstall = ''
    mkdir -p $out/{lib,include,share/${pname}}

    # Move libraries if they exist
    if [ -d $out/share/${pname}-${version}/lib ]; then
      mv $out/share/${pname}-${version}/lib/* $out/lib/
      rm -rf $out/share/${pname}-${version}/lib
    fi

    # Install message definitions
    mkdir -p $out/share/${pname}/msg
    cp -r msg/*.msg $out/share/${pname}/msg/

    # Install generated headers
    mkdir -p $out/include/${pname}/msg
    if ls build/livox_interfaces2/include/livox_interfaces2/msg/* >/dev/null 2>&1; then
      cp -r build/livox_interfaces2/include/livox_interfaces2/msg/* $out/include/${pname}/msg/
    else
      echo "WARNING: No generated headers found in build/livox_interfaces2/include/livox_interfaces2/msg/"
    fi
  '';

  meta = with lib; {
    description = "Livox ROS Driver 2.0 for Livox LiDAR";
    homepage = "https://github.com/Livox-SDK/livox_ros_driver2";
    license = licenses.mit;
    maintainers = with maintainers; [ ];
    platforms = platforms.linux;
  };
}
