{
  stdenv,
  cleanCmakeSource,
  cmake,
  gtest,
}:

stdenv.mkDerivation rec {
  pname = "hi-can";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  nativeBuildInputs = [
    cmake
    gtest
  ];
  doCheck = true;
}
