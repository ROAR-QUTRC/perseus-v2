{ stdenv, cmake }:

stdenv.mkDerivation rec {
  pname = "hi-can-lib";
  version = "0.0.0";

  src = builtins.path {
    path = ./.;
    name = pname;
  };

  nativeBuildInputs = [ cmake ];
}
