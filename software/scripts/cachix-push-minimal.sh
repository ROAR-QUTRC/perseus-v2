#! /usr/bin/env nix-shell
#! nix-shell -p cachix jq -i bash
# shellcheck shell=bash

# WARNING: This script is intended to be used only in CI/CD.

set -e

# push built packages
nix build --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar

# push input flakes
nix flake archive --json | jq -r '.path,(.inputs|to_entries[].value.path)' | cachix push qutrc-roar

# push dev shell environment
nix develop --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*
