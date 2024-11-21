{
  lib,
  p7zip,
  writeShellScriptBin,
}:
writeShellScriptBin "decompress" ''
  ${lib.getExe p7zip} x $1 -o$2
''
