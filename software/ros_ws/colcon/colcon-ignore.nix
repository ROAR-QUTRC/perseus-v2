{ stdenv }:
stdenv.mkDerivation {
  dontUnpack = true;
  name = "colcon-ignore";
  installPhase = ''
    mkdir -p $out
    touch $out/COLCON_IGNORE
  '';
}
