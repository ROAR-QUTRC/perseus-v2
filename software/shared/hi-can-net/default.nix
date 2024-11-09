{
  stdenv,
  cleanCmakeSource,
  cmake,
  hi-can-lib,
}:

stdenv.mkDerivation rec {
  pname = "hi-can-net";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  buildInputs = [ hi-can-lib ];
  nativeBuildInputs = [ cmake ];
}
