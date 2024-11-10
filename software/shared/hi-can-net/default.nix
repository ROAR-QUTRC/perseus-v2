{
  stdenv,
  cleanCmakeSource,
  cmake,
  hi-can,
}:

stdenv.mkDerivation rec {
  pname = "hi-can-net";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  buildInputs = [ hi-can ];
  nativeBuildInputs = [ cmake ];
}
