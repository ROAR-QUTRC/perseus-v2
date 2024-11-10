{
  stdenv,
  cleanCmakeSource,
  cmake,
  hi-can-raw,
}:

stdenv.mkDerivation rec {
  pname = "net-can-server";
  version = "0.0.0";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  nativeBuildInputs = [ cmake ];
  buildInputs = [ hi-can-raw ];
}
