final: prev: {
  autonomy = final.callPackage ./autonomy.nix { };
  input-devices = final.callPackage ./input-devices.nix { };
  perseus = final.callPackage ./perseus.nix { };
  perseus-description = final.callPackage ./perseus-description.nix { };
  perseus-hardware = final.callPackage ./perseus-hardware.nix { };
  perseus-navigator = final.callPackage ./perseus-navigator.nix { };
  perseus-sensors = final.callPackage ./perseus-sensors.nix { };
  perseus-simulation = final.callPackage ./perseus-simulation.nix { };
}
