{
  stdenv,
  p7zip,
  docs,
}:
stdenv.mkDerivation {
  name = "roar-docs-compressed";
  buildInputs = [
    docs
    p7zip
  ];
  src = docs;
  installPhase = ''
    mkdir -p $out
    7z a -t7z -m0=lzma -mx=9 -mfb=64 -md=32m -ms=on $out/docs.7z .
  '';
}
