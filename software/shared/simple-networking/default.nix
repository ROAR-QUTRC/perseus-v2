{
  stdenv,
  cleanCmakeSource,
  cmake,
  fd-wrapper,
  ptr-wrapper,
}:

stdenv.mkDerivation rec {
  pname = "simple-networking";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  # due to header files, these need to propagate
  propagatedBuildInputs = [
    fd-wrapper
    ptr-wrapper
  ];
  nativeBuildInputs = [ cmake ];
}
