#! /usr/bin/env bash
set -e

shopt -s globstar # Enable globstar for recursive globbing

# cd to the repo root
cd "$(git rev-parse --show-toplevel)"

rm -rf ./**/result
rm -rf ./**/build
rm -rf ./**/log
rm -rf ./**/install
rm -rf ./**/generated
