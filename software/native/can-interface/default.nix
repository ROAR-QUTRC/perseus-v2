{
  stdenv,
  cmake,
  hi-can-lib,
}:

stdenv.mkDerivation rec {
  pname = "can-interface";
  version = "0.0.0";

  srcs = (
    builtins.path {
      path = ./.;
      name = pname;
    }
  );

  nativeBuildInputs = [ cmake ];
  buildInputs = [ hi-can-lib ];
}
