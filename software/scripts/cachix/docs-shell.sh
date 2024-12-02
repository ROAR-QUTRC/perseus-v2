# /usr/bin/env bash
# WARNING: This script is intended to be used only through `nix run`.

# make script brittle - fail on any error
set -euo pipefail

# push dev shell environment
nix develop .#docs --profile roar-devenv -c true
cachix push roar-qutrc roar-devenv

rm roar-devenv*
