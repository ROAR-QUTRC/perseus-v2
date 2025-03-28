{
  stdenv,
  cleanCmakeSource,
  cmake,
  fd-wrapper,
  hi-can,
}:

stdenv.mkDerivation rec {
  pname = "hi-can-raw";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  # due to header files, these need to propagate
  propagatedBuildInputs = [
    fd-wrapper
    hi-can
  ];
  nativeBuildInputs = [ cmake ];
}
