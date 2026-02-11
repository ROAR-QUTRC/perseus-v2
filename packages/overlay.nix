# packages/overlay.nix
final: prev: {
  groot2 = final.callPackage ./groot2 { };
  livox-sdk2 = final.callPackage ./livox-sdk2 { };
  freeimage =
    let
      libjpeg-static = (final.libjpeg.override { enableStatic = true; }).overrideAttrs (
        {
          patches ? [ ],
          postFixup ? "",
          ...
        }:
        {
          version = "3.0.4";

          src = prev.fetchFromGitHub {
            owner = "libjpeg-turbo";
            repo = "libjpeg-turbo";
            rev = "3.0.4";
            hash = "sha256-ZNqhOfZtWcMv10VWIUxn7MSy4KhW/jBrgC1tUFKczqs=";
          };
          patches = patches ++ [
            ./libjpeg/0001-Compile-transupp.c-as-part-of-the-library.patch
            ./libjpeg/0002-Make-exported-symbols-in-transupp.c-weak.patch
          ];
          postFixup = postFixup + ''
            moveToOutput include/transupp.h $dev
          '';
        }
      );
    in
    (final.callPackage ./freeimage { }).override {
      libjpeg = libjpeg-static;
    };
  python312 = prev.python312.override {
    packageOverrides = pyFinal: pyPrev: {
      pyside2 = pyPrev.pyside2.overrideAttrs (
        {
          patches ? [ ],
          ...
        }:
        {
          # For some reason setting the CMake limited API option doesn't work, so patch it to remove the option altogether
          patches = patches ++ [ ./pyside2/limited-api.patch ];
        }
      );
    };
  };
}
