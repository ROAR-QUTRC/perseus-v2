# Documentation

## How to build

Unless you're making major edits to the raw markdown/reST files, or you're fiddling with the build toolchain, you probably don't need to build them locally - let the continuous deployment take care of it for you!
However, if you do need to test things locally, the Pixi `docs` environment provides everything needed.

### Building

0. Run a `git pull` - since CI/CD runs Intersphinx inventory updates, you need to make sure your repo is up to date before building!
1. Run `pixi install -e docs` (first time only, or after `pixi.toml`/`pixi.lock` changes) to resolve and fetch the `docs` environment.
2. Run `pixi run -e docs docs` - this invokes `uv run --project pyproject sphinx-build -b html source build/html` and outputs the built site to `docs/build/html`.

:::{note}
draw.io figure generation (building `.drawio` sources into SVGs) was part of the old Nix-based docs shell and has not yet been ported to Pixi - see the `[feature.docs.dependencies]` comment in `pixi.toml`. If you need to update figures, you'll need `drawio` available some other way for now.
:::

Note that depending on the edits you made, you may also have to remove `docs/build` before rebuilding.
Sphinx is smart enough to only rebuild the files which have changed, which is normally fine - except for when changes propagate to other files.
One such example is adding new files which appear in the navigation tree - although this affects _every_ page, since they all have the navigation sidebar, only files whose `toctree`s have changed will be updated.
As such, the new file and its associated heading will not be visible in navigation until a full rebuild after removing `docs/build`.

### Python environment management with `uv`

Before managing python dependencies, you will first need to cd to the `docs/pyproject` directory.
This is because the `pyproject.toml` and `uv.lock` files (defining the Python project) live in this directory so that only changes to these specific files cause a rebuild of the Python workspace.

To:

- Add dependencies: `uv add DEPENDENCY`
- Remove dependencies: `uv remove DEPENDENCY`
- Update the package versions in the lockfile: `uv lock --update`

These changes take effect the next time `pixi run -e docs docs` (or `uv run ...` directly) resolves the environment - no separate shell re-entry step is needed.
Alternatively, you can also run `uv run COMMAND` to run a command in the currently specified Python environment.
However, as previously mentioned, `uv` can only be run from the directory containing the `pyproject.toml` file, which limits its usefulness.
For further usage, consult the `uv` [usage documentation](https://docs.astral.sh/uv/reference/cli/) - in particular you may find the `venv` command useful.

## Serving the built docs

Whilst simply opening the built files from a file browser is fine 99% of the time, some JavaScript may not run correctly without being loaded from a HTTP server for some reason.
Python's built-in HTTP server is always available in the `docs` Pixi environment and works well for this:

```console
pixi run -e docs python -m http.server -d docs/build/html 8080
```

Then view `localhost:8080` (or the loopback address `127.0.0.1:8080`) in a browser - you may have to experiment with which one loads correctly, as sometimes one of these won't work for certain features.
To serve on a different port than `8080`, change the trailing port number in the command above.
