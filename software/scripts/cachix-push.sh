# /usr/bin/env bash
# WARNING: This script is intended to be used only through `nix run`.

set -euo pipefail

# cd to the repo root
cd "$(git rev-parse --show-toplevel)"

# push built packages
nix build --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build .#docs --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build .#simulation --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar

# push useful tooling and the like
nix build .#pkgs.scripts.cachix-push --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build .#pkgs.scripts.cachix-push-minimal --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build .#pkgs.scripts.clean --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar

# push input flakes
nix flake archive --json | jq -r '.path,(.inputs|to_entries[].value.path)' | cachix push qutrc-roar

# push dev shell environment
nix develop --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*

nix develop .#docs --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*

nix develop .#simulation --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*
