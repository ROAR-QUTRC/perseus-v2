{ stdenv, cmake }:

stdenv.mkDerivation rec {
  pname = "hi-can-lib";
  version = "0.0.1";

  src = builtins.path {
    path = ./.;
    name = pname;
  };

  nativeBuildInputs = [ cmake ];
}
