# Documentation Maintenance

This documentation is loosely based off of Rohit Gaswami's blog posts documenting [setting up](https://rgoswami.me/posts/doc-cpp-dox-sph-exhale/) Sphinx, Breathe, and Exhale, as well as the follow-up detailing [publishing](https://rgoswami.me/posts/pub-doc-cpp-dox-sph-nix/) them with Nix.

## How to build

Unless you're making edits to the raw markdown/reST files, you should never need to build them locally - let the continuous deployment take care of it for you! However, if you do need to test edits locally, the process is as follows:

From the repo root:

1. Run `nix develop .#docs`
2. Run `cd docs`
3. Whenever you want to rebuild the docs, run `make clean && make html` - they will be built to `build/html`
4. Optionally, if you want to serve the docs instead of opening the file locally, open a new terminal and run `nix run nixpkgs#darkhttpd -- ./docs/build/html` ({keys}`ctrl+c` to terminate) and open `127.0.0.1:8080`.
