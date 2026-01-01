final: prev: {
  autonomy = final.callPackage ./autonomy.nix { };
  input-devices = final.callPackage ./input-devices.nix { };
  perseus = final.callPackage ./perseus.nix { };
  perseus-autonomy-bridge = final.callPackage ./perseus-autonomy-bridge.nix { };
  perseus-autonomy-interfaces = final.callPackage ./perseus-autonomy-interfaces.nix { };
  perseus-can-if = final.callPackage ./perseus-can-if.nix { };
  perseus-description = final.callPackage ./perseus-description.nix { };
  perseus-hardware = final.callPackage ./perseus-hardware.nix { };
  perseus-input = final.callPackage ./perseus-input.nix { };
  perseus-input-config = final.callPackage ./perseus-input-config.nix { };
  perseus-lite = final.callPackage ./perseus-lite.nix { };
  perseus-lite-description = final.callPackage ./perseus-lite-description.nix { };
  perseus-lite-hardware = final.callPackage ./perseus-lite-hardware.nix { };
  perseus-payloads = final.callPackage ./perseus-payloads.nix { };
  perseus-sensors = final.callPackage ./perseus-sensors.nix { };
  perseus-simulation = final.callPackage ./perseus-simulation.nix { };
  teleop-diagnostics = final.callPackage ./teleop-diagnostics.nix { };
}
