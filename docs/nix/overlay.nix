final: prev:
let
  doxygen = prev.doxygen.overrideAttrs (
    { ... }:
    rec {
      version = "1.12.0";
      patches = [ ];

      src = prev.fetchFromGitHub {
        owner = "doxygen";
        repo = "doxygen";
        rev = "Release_${prev.lib.replaceStrings [ "." ] [ "_" ] version}";
        sha256 = "sha256-4zSaM49TjOaZvrUChM4dNJLondCsQPSArOXZnTHS4yI=";
      };
    }
  );
in
{
  inherit doxygen;
}
