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
  # clean up generated figures + code docs
  rm -rf "${output-dir}"
  # and old Doxygen output
  rm -rf "../build/doxygen"
  mkdir -p "${output-dir}"
  cp -a ${figures}/. "${output-dir}"
  # nix store is read-only by default, so fix that after copying
  chmod -R +w "${output-dir}"
''
