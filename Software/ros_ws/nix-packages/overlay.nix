self: super: {
  autonomy = super.callPackage ./autonomy.nix { };
  hardware-interfaces = super.callPackage ./hardware-interfaces.nix { };
  input-controllers = super.callPackage ./input-controllers.nix { };
  perseus-bringup = super.callPackage ./perseus-bringup.nix { };
}
