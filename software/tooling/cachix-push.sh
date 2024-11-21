#! /usr/bin/env nix-shell
#! nix-shell -p cachix jq -i bash

set -e

# cd to the Software root so it picks the flake properly
SCRIPT_DIR="$(dirname "$(readlink -f $0)")"
cd $SCRIPT_DIR/..

# push built packages
nix build .#rosCore --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build .#docs --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build .#roverSim --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar

# push input flakes
nix flake archive --json | jq -r '.path,(.inputs|to_entries[].value.path)' | cachix push qutrc-roar

# push dev shell environment
nix develop --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*

nix develop .#docs --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*

nix develop .#sim --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*
