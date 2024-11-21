{
  writeShellScriptBin,
  cd-docs-source,
  figures,
}:
let
  output-dir = "generated";
in
writeShellScriptBin "roar-docs-setup" ''
  set -e

  source ${cd-docs-source}
  mkdir -p "${output-dir}"
  cp -a ${figures}/. "${output-dir}"
  # nix store is read-only by default, so fix that after copying
  chmod -R +w "${output-dir}"
''
