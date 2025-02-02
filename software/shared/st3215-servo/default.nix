{
  stdenv,
  cleanCmakeSource,
  cmake,
  boost,
}:

stdenv.mkDerivation rec {
  pname = "st3215-servo";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  # due to header files, these need to propagate
  propagatedBuildInputs = [
    boost
  ];
  nativeBuildInputs = [ cmake ];
}
