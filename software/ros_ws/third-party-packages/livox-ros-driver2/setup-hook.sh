# Setup hook for livox-ros-driver2
addLivoxDriverToEnv() {
  # Add to AMENT_PREFIX_PATH
  addToSearchPath AMENT_PREFIX_PATH $1/share

  # Add to library path
  addToSearchPath LD_LIBRARY_PATH $1/lib

  # Add to CMAKE paths
  addToSearchPath CMAKE_PREFIX_PATH $1

  # Add to include paths
  addToSearchPath CPATH $1/include
}

envHooks+=(addLivoxDriverToEnv)
