{
  stdenv,
  cleanCmakeSource,
  cmake,
}:

stdenv.mkDerivation rec {
  pname = "hi-can-lib";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  nativeBuildInputs = [ cmake ];
}