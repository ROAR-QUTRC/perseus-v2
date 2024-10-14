#! /usr/bin/env bash
shopt -s globstar  # Enable globstar for recursive globbing
SCRIPT_DIR="$(dirname "$(readlink -f $0)")"
PROJECT_ROOT="$SCRIPT_DIR/../.."
cd $PROJECT_ROOT

rm -rf "$PROJECT_ROOT"/**/result
rm -rf "$PROJECT_ROOT"/**/build
rm -rf "$PROJECT_ROOT"/**/log
rm -rf "$PROJECT_ROOT"/**/install