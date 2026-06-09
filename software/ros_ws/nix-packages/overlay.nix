final: prev: {
  autonomy = final.callPackage ./autonomy.nix { };
  input-devices = final.callPackage ./input-devices.nix { };
  pcl-to-lsr = final.callPackage ./pcl-to-lsr.nix { };
  perseus-autonomy-bridge = final.callPackage ./perseus-autonomy-bridge.nix { };
  perseus-bt-nodes = final.callPackage ./perseus-bt-nodes.nix { };
  perseus-input = final.callPackage ./perseus-input.nix { };
  perseus-input-config = final.callPackage ./perseus-input-config.nix { };
  perseus-interfaces = final.callPackage ./perseus-interfaces.nix { };
  perseus-lite = final.callPackage ./perseus-lite.nix { };
  perseus-lite-description = final.callPackage ./perseus-lite-description.nix { };
  perseus-lite-hardware = final.callPackage ./perseus-lite-hardware.nix { };
  perseus-lite-missions = final.callPackage ./perseus-lite-missions.nix { };
  perseus-lite-simulation = final.callPackage ./perseus-lite-simulation.nix { };
  perseus-sensors = final.callPackage ./perseus-sensors.nix { };
  perseus-vision = final.callPackage ./perseus-vision.nix { };
  teleop-diagnostics = final.callPackage ./teleop-diagnostics.nix { };
}
