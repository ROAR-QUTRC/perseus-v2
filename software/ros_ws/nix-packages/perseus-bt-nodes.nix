{
  lib,
  buildRosPackage,
  ament-cmake,
  behaviortree-cpp,
  geometry-msgs,
}:
buildRosPackage rec {
  pname = "ros-jazzy-perseus-bt-nodes";
  version = "0.0.0";

  src = ./../src/perseus_bt_nodes;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    behaviortree-cpp
    geometry-msgs
  ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = "Custom BehaviorTree nodes for Perseus";
    license = with lib.licenses; [ mit ];
  };
}
