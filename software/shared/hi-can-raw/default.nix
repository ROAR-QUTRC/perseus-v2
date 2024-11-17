{
  stdenv,
  cleanCmakeSource,
  cmake,
  fd-wrapper,
  hi-can-lib,
}:

stdenv.mkDerivation rec {
  pname = "hi-can-raw";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  buildInputs = [
    fd-wrapper
    hi-can-lib
  ];
  nativeBuildInputs = [ cmake ];
}
