#! /usr/bin/env bash

# If you're changing this file, make sure to read the systems/software/scripts docs

set -euo pipefail

shopt -s globstar # Enable globstar for recursive globbing

# cd to the repo root
cd "$(git rev-parse --show-toplevel)"

rm -rf ./**/result
rm -rf ./**/build
rm -rf ./**/log
rm -rf ./**/install
rm -rf ./**/generated
