{
  stdenv,
  cleanCmakeSource,
  cmake,
  fd-wrapper,
  hi-can,
}:

stdenv.mkDerivation rec {
  pname = "hi-can-net";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  buildInputs = [
    fd-wrapper
    hi-can
  ];
  nativeBuildInputs = [ cmake ];
}
