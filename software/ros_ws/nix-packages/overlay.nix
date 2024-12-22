self: super: {
  autonomy = super.callPackage ./autonomy.nix { };
  input-devices = super.callPackage ./input-devices.nix { };
  perseus = super.callPackage ./perseus.nix { };
  perseus-description = super.callPackage ./perseus-description.nix { };
  perseus-hardware = super.callPackage ./perseus-hardware.nix { };
  perseus-sensors = super.callPackage ./perseus-sensors.nix { };
}
