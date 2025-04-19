# Documentation

## How to build

Unless you're making major edits to the raw markdown/reST files, or you're fiddling with the build toolchain, you probably don't need to build them locally - let the continuous deployment take care of it for you!
However, if you do need to test things locally, you have two options.
The first is to use `nix build` - this is ideal if you're making small edits to the documentation and you want to check that everything's building properly before pushing to GitHub.
The second is to enter a _development shell_ using `nix develop` - if you want slightly faster builds, you're modifying Python dependencies, or you're tinkering with the build toolchain, this is what you'll need.

### Using `nix build`

To build the documentation, simply run `nix build .#docs` - the output will be in the `result` symlink in the directory where you ran the command.

### Using `nix develop`

#### Building

0. Run a `git pull` - since CI/CD runs Intersphinx inventory updates, you need to make sure your repo is up to date before building!
1. Run `nix develop .#docs`
2. Run `nix run .#docs.setup` - this will build the draw.io figures to SVGs and copy them into `/docs/source/generated`.
3. `cd` to the `/docs` directory
4. Whenever you want to rebuild the docs, run `make html` - they will be output in `build/html`.

Note that depending on the edits you made, you may also have to run `make clean` before rebuilding.
Sphinx is smart enough to only rebuild the files which have changed, which is normally fine - except for when changes propagate to other files.
One such example is adding new files which appear in the navigation tree - although this affects _every_ page, since they all have the navigation sidebar, only files whose `toctree`s have changed will be updated.
As such, the new file and its associated heading will not be visible in navigation until a full rebuild after a `clean`.

#### Python environment management with `uv`

Before managing python dependencies, you will first need to cd to the `/docs/pyproject` directory.
This is because the `pyproject.toml` and `uv.lock` files (defining the Python project) have been moved to this directory so that only changes to these specific files cause a rebuild of the Python workspace.

To:

- Add dependencies: `uv add DEPENDENCY`
- Remove dependencies: `uv remove DEPENDENCY`
- Update the package versions in the lockfile: `uv lock --update`

You will have to exit and re-enter the dev shell for any of these changes to take effect, as the Python environment is built and applied upon entering the dev shell.
Alternatively, you can also run `uv run COMMAND` to run a command in the currently specified Python environment.
However, as previously mentioned, `uv` can only be run from the directory containing th `pyproject.toml` file, which limits its usefulness.
For further usage, consult the `uv` [usage documentation](https://docs.astral.sh/uv/reference/cli/) - in particular you may find the `venv` command useful (though at that point you may as well just exit and re-enter the shell to rebuild the environment with Nix anyway).

## Serving the built docs

`darkhttpd` is an excellent fast and lightweight HTTP server, and is excellent for quickly testing that the docs are displaying properly.
Whilst simply opening the built files from a file browser is fine 99% of the time, some JavaScript may not run correctly without being loaded from a HTTP server for some reason.
Run `nix run nixpkgs#darkhttpd -- DOCS_LOCATION` to start serving the documentation ({keys}`ctrl+c` to terminate).
You may have to experiment with using either `localhost:8080` or the loopback address `127.0.0.1:8080` - sometimes one of these won't load correctly and certain features won't work.

- Using `nix build`: `nix run nixpkgs#darkhttpd -- ./result/html`
- Building with the dev shell: `nix run nixpkgs#darkhttpd -- ./docs/build/html` (assuming you're in the repository root)

To serve on a different port than `8080`, add the `--port PORT` flag to the end of the command.
