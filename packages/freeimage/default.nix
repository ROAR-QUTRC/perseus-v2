{ stdenv, fetchFromGitHub, libjpeg, libpng, libtiff, git, cmake, ... }:
stdenv.mkDerivation {
  name = "FreeImageRe";
  version = "0.3";
  src = fetchFromGitHub {
    owner = "agruzdev";
    repo = "FreeImageRe";
    rev = "v0.3";
    hash = "sha256-3Akl6pmmTuBPr/df8O7K3iNwzXoeAXKp8VSInv2P/t0=";
  };
  buildtype = "cmake";
  buildInputs = [ ];
  propagatedBuildInputs = [ libjpeg libpng libtiff git ];
  nativeBuildInputs = [ cmake ];
  cmakeFlags = [
    "-DEXTERNALPROJECT_UNPACK_IN_BINARY_DIR=ON"
  ];
}