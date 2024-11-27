{
  stdenvNoCC,
  p7zip,
  docs,
}:
stdenvNoCC.mkDerivation {
  name = "roar-docs-compressed";
  buildInputs = [ p7zip ];
  src = docs;
  # run 7zip with "extreme" compression settings
  # a = add files to archive
  # next arg is output dir
  # and finally the input dir (which is the source dir), which is the built docs
  installPhase = ''
    mkdir -p $out
    7z a -t7z -m0=lzma -mx=9 -mfb=64 -md=32m -ms=on $out/docs.7z .
  '';
}
