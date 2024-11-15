# Documentation Maintenance

This documentation is loosely based off of Rohit Gaswami's blog posts documenting [setting up](https://rgoswami.me/posts/doc-cpp-dox-sph-exhale/) Sphinx, Breathe, and Exhale, as well as the follow-up detailing [publishing](https://rgoswami.me/posts/pub-doc-cpp-dox-sph-nix/) them with Nix.

## How to build

Unless you're making edits to the raw markdown/reST files, you should never need to build them locally - let the continuous deployment take care of it for you! However, if you do need to test edits locally, the process is as follows:

1. Install [`uv`](https://docs.astral.sh/uv/)
2. Install Doxygen
3. If you're using Nix, you can run `nix shell nixpkgs/nixpkgs-unstable#uv nixpkgs#doxygen` to enter a shell with these tools instead - note that `uv` comes from `nixpkgs-unstable`, as we need the very latest version!
4. `cd` to the `docs/` folder
5. Run `doxygen` to generate all source code
6. Run `uv run make html` to build the docs with Sphinx
7. That's it! The documentation is now built and available under the `html/` directory.
