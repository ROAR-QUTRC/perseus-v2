{ writeShellScript, git }:
# Utility script which changes directory to the /docs/source directory of the git repository
# Note: Can only be used in nix run scripts as it relies on running within a git repository
# (which is not present when building, but that's not an issue because we know what the source directory is there)
writeShellScript "roar-docs-inventory" ''
  # cd to root of git repo
  cd $(${git}/bin/git rev-parse --show-toplevel)
  # cd to docs
  cd docs/source
''
