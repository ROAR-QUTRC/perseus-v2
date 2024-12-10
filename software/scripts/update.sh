#!/usr/bin/env bash
set -euo pipefail

# cd to the repo root
cd "$(git rev-parse --show-toplevel)"

nix flake update
nix run .#tools.treefmt-write-config
