# Fetches intersphinx inventory files
{
  rosDistro,
  writeShellScriptBin,
  cd-docs-source,
}:
writeShellScriptBin "roar-docs-fetch-inventories" ''
  set -e

  INVENTORY_LOCATION="intersphinx"

  source ${cd-docs-source}

  # get intersphinx inventories
  # clear existing
  rm -rf "$INVENTORY_LOCATION"

  # get the files
  echo "Fetching intersphinx inventories"
  curl --parallel \
    --create-dirs --output-dir "$INVENTORY_LOCATION" \
    --output python.inv https://docs.python.org/3/objects.inv \
    --variable rosDistro="${rosDistro}" \
    --output ros.inv --expand-url https://docs.ros.org/en/{{rosDistro:trim:url}}/objects.inv

  if [[ "$*" == *--no-commit* ]]; then
      echo "Will not commit changes"
      exit 0
  fi

  if ! git diff --cached --quiet >/dev/null; then
      HAS_GIT_STAGING=1
      echo "Stashing staged changes"
      git stash push -S >/dev/null
  fi

  cleanup() {
    if [[ -v HAS_GIT_STAGING ]]; then
        echo "Restoring git stash"
        git stash pop >/dev/null
    fi
  }
  trap cleanup EXIT

  echo "Staging changes"
  git add "$INVENTORY_LOCATION"
  echo "Committing changes"
  git commit -m "$(date +%z:%Y-%m-%dT%H:%M:%S) Intersphinx inventory fetch" >/dev/null
''