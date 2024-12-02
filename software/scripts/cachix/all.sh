#! /usr/bin/env bash
# WARNING: This script is intended to be used only through `nix run`.

set -euo pipefail

# cd to the repo root
cd "$(git rev-parse --show-toplevel)"

# push built packages
nix build --json | jq -r '.[].outputs | to_entries[].value' | cachix push roar-qutrc
nix build .#docs --json | jq -r '.[].outputs | to_entries[].value' | cachix push roar-qutrc
nix build .#simulation --json | jq -r '.[].outputs | to_entries[].value' | cachix push roar-qutrc

# push useful tooling and the like
nix build .#pkgs.scripts.cachix.all --json | jq -r '.[].outputs | to_entries[].value' | cachix push roar-qutrc
nix build .#pkgs.scripts.cachix.build --json | jq -r '.[].outputs | to_entries[].value' | cachix push roar-qutrc
nix build .#pkgs.scripts.cachix.shell --json | jq -r '.[].outputs | to_entries[].value' | cachix push roar-qutrc
nix build .#pkgs.scripts.clean --json | jq -r '.[].outputs | to_entries[].value' | cachix push roar-qutrc

# push input flakes
nix flake archive --json | jq -r '.path,(.inputs|to_entries[].value.path)' | cachix push roar-qutrc

# push dev shell environment
nix develop --profile roar-devenv -c true
cachix push roar-qutrc roar-devenv

rm roar-devenv*

nix develop .#docs --profile roar-devenv -c true
cachix push roar-qutrc roar-devenv

rm roar-devenv*

nix develop .#simulation --profile roar-devenv -c true
cachix push roar-qutrc roar-devenv

rm roar-devenv*
