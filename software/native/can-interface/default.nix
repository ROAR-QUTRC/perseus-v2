{
  stdenv,
  cleanCmakeSource,
  cmake,
  hi-can-lib,
}:

stdenv.mkDerivation rec {
  pname = "can-interface";
  version = "0.0.0";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  nativeBuildInputs = [ cmake ];
  buildInputs = [ hi-can-lib ];
}
