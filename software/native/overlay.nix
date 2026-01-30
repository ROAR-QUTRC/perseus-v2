final: prev: {
  examples = final.lib.packagesFromDirectoryRecursive {
    callPackage = final.callPackage;
    directory = ./examples;
  };
  testing = final.lib.packagesFromDirectoryRecursive {
    callPackage = final.callPackage;
    directory = ./testing;
  };
}
