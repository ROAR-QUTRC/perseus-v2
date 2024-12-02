# /usr/bin/env bash
# WARNING: This script is intended to be used only through `nix run`.

# make script brittle - fail on any error
set -euo pipefail

# push built packages
nix build --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar

# push input flakes
nix flake archive --json | jq -r '.path,(.inputs|to_entries[].value.path)' | cachix push qutrc-roar

# push dev shell environment
nix develop --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*
