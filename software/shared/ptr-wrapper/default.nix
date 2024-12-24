{
  stdenv,
  cleanCmakeSource,
  cmake,
}:

stdenv.mkDerivation rec {
  pname = "ptr-wrapper";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  nativeBuildInputs = [ cmake ];
}
