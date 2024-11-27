{
  writeShellScriptBin,
  cd-docs-source,
  figures,
}:
# this script copies the built figures to the /docs/source/generated directory
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
