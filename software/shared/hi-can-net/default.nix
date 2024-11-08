{
  stdenv,
  cmake,
  hi-can-lib,
}:

stdenv.mkDerivation rec {
  pname = "hi-can-net";
  version = "0.0.1";

  src = builtins.path {
    path = ./.;
    name = pname;
  };

  buildInputs = [ hi-can-lib ];
  nativeBuildInputs = [ cmake ];
}
