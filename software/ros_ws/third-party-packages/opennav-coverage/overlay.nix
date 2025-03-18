final: prev: {
  opennav-coverage = final.callPackage ./opennav-coverage.nix { };
  opennav-coverage-bt = final.callPackage ./opennav-coverage-bt.nix { };
  opennav-coverage-demo = final.callPackage ./opennav-coverage-demo.nix { };
  opennav-coverage-msgs = final.callPackage ./opennav-coverage-msgs.nix { };
  opennav-coverage-navigator = final.callPackage ./opennav-coverage-navigator.nix { };
  opennav-row-coverage = final.callPackage ./opennav-row-coverage.nix { };
}
