self: super: {
  autonomy = super.callPackage ./autonomy.nix { };
  hardware-interfaces = super.callPackage ./hardware-interfaces.nix { };
  input-devices = super.callPackage ./input-devices.nix { };
  perseus = super.callPackage ./perseus.nix { };
}
