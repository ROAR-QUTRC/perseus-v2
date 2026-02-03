{ stdenv, fetchFromGitHub, python3, ... }:
stdenv.mkDerivation {
  pname = "pyvesc";
  version = "1.0";
  src = fetchFromGitHub {
    owner = "LiamBindle";
    repo = "PyVESC";
    rev = "sha1-d2f2076afe91dafa920a38a437528819697e05e3";
    hash = "sha1-d2f2076afe91dafa920a38a437528819697e05e3";
  };
  nativeBuildInputs = [ python3 ];
}