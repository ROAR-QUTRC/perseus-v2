final: prev: {
  examples = final.lib.packagesFromDirectoryRecursive {
    callPackage = final.callPackage;
    directory = ./examples;
  };
}
