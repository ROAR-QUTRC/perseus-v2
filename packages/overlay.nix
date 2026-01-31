# packages/overlay.nix
final: prev: {
  groot2 = final.callPackage ./groot2 { };
  livox-sdk2 = final.callPackage ./livox-sdk2 { };
  freeimage = final.callPackage ./freeimage { };
  libjpeg-src = final.callPackage ./libjpeg { };
  python312 = prev.python312.override {
    packageOverrides = pyFinal: pyPrev: {
      pyside2 = pyPrev.pyside2.overrideAttrs (
        {
          patches ? [ ],
          ...
        }:
        {
          patches = patches ++ [ ./pyside2/limited-api.patch ];
        }
      );
      pyside6 = { };
    };
  };
}
