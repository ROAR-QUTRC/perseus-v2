final: prev: {
  autonomy = final.callPackage ./autonomy.nix { };
  input-devices = final.callPackage ./input-devices.nix { };
  perseus = final.callPackage ./perseus.nix { };
  perseus-can-if = final.callPackage ./perseus-can-if.nix { };
  perseus-description = final.callPackage ./perseus-description.nix { };
  perseus-hardware = final.callPackage ./perseus-hardware.nix { };
  perseus-input = final.callPackage ./perseus-input.nix { };
  perseus-input-config = final.callPackage ./perseus-input-config.nix { };
  perseus-payloads = final.callPackage ./perseus-payloads.nix { };
  perseus-sensors = final.callPackage ./perseus-sensors.nix { };
  perseus-simulation = final.callPackage ./perseus-simulation.nix { };
  perseus-teleop = final.callPackage ./perseus-teleop.nix { };
<<<<<<< HEAD
  perseus-vision = final.callPackage ./perseus-vision.nix {
  };
=======
  perseus-vision = final.callPackage ./perseus-vision.nix { };
>>>>>>> 349a43874b7cc2752ee3de159da637e11fd7e4aa
}
