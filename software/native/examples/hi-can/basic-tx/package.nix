{
  stdenv,
  cleanCmakeSource,
  cmake,
  hi-can-raw,
}:

stdenv.mkDerivation rec {
  pname = "hi_can_basic_write";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  nativeBuildInputs = [ cmake ];
  buildInputs = [ hi-can-raw ];
}
