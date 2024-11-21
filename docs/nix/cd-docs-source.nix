{ writeShellScript }:
writeShellScript "roar-docs-inventory" ''
  # cd to docs/source assuming they exist (we're not already there)
  if [ -d docs ]; then
    cd docs
  fi
  if [ -d source ]; then
    cd source
  fi
''
