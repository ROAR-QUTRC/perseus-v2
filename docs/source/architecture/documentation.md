# Documentation

Good documentation is key to a project's long-term success.
As such, the documentation pipeline which ends up generating these very webpages is explained in this document.
If you're looking to start _writing_ documentation, check out the [maintenance](project:/maintenance/documentation.md) page after reading through this one.

## Tech Stack

### Core

This documentation is built using <inv:sphinx:std#index> with a number of extensions.

#### Markdown Support

If you inspect the raw documentation, you'll notice it's written in Markdown, not the standard reStructuredText (ReST) that Sphinx expects - this is thanks to the [MyST Parser](inv:myst#index).
This extension extends Sphinx to add Markdown parsing support.
It also comes with a number of built-in [extensions](inv:myst#syntax/optional) which can be enabled to extend its functionality on top of standard Markdown support.
If you're curious as to what this documentation uses, check out the list of them in `docs/source/conf.py`.

#### Generated Documentation

Code (API) documentation is provided with Doxygen through the use of [Breathe](inv:breathe#index), which acts as a bridge from the output Doxygen provides to ReST for Sphinx.
Although it's not leveraged much in this documentation, you can reference specific [files](inv:breathe#file), [classes](inv:breathe#class), or [methods](inv:breathe#function), and [more](inv:breathe#directives) using Breathe.
Next is the [Exhale](inv:exhale#index) extension, which provides the Doxygen-style namespace, class, and file [lists](project:/generated/index.rst).
It's also set up to automatically run Doxygen, so there's no need for a standard configuration `Doxyfile` - the modifications to the standard configuration are provided with [configuration](inv:exhale#*exhaleDoxygenStdin) options on top of Exhale's [defaults](inv:exhale#*DEFAULT_DOXYGEN_STDIN_BASE).

#### Theming

After these core extensions, comes theming.
As it says in the footer, this site is built using [Sphinx-Immaterial](inv:sphinx-immaterial#index).
Whilst you shouldn't need to modify its configuration unless the documentation website is actively broken, it is very well documented if you need to do so.
In this documentation, we also make use of some of its excellent extensions: [`sphinx_immaterial.kbd_keys`](inv:sphinx-immaterial:std#keys) for pretty keyboard keys (like {keys}`ctrl+c`!), [`sphinx_immaterial.apidoc.cpp.cppreference`](inv:sphinx-immaterial#apidoc/cpp/cppreference) for automatic links to cppreference.com, and finally [`sphinx_immaterial.graphviz`](inv:sphinx-immaterial#graphviz) for lovely theme-integrated [Graphviz](https://graphviz.org/) diagrams - though support for <inv:sphinx-immaterial#mermaid_diagrams> is also baked in by default if that's more your style.

#### Utility Extensions

We also include [`sphinxcontrib.jquery`](https://github.com/sphinx-contrib/jquery) - some plugins expect to run on ReadTheDocs' website which always has JQuery available.
It's also useful for developing custom JavaScript to enhance the pages if needed.
In addition, the [`sphinx_tippy`](https://sphinx-tippy.readthedocs.io/en/latest/) extension is also included to add preview tooltips when you hover on links.
If it's not listed here, all enabled extensions have a comment explaining what they do in the `conf.py` file.

### Sandboxing

Next, there's the tweaks to make the docs build with no internet access.
Since this project is built using Nix (more on that in the [next section](#build-infrastructure)), the entire software stack needs to run offline and therefore requires some extra configuration.
The first offender is the Sphinx-Immaterial theme itself - it sources its font ([Roboto](https://fonts.google.com/specimen/Roboto) by default) from Google's CDNs - this is obviously incompatible with the sandboxing that Nix applies!
As such, its <inv:sphinx-immaterial#font> option has been set to `False` and the fonts are "self-hosted" in the `_static/fonts` directory.
The other item on the list is [intersphinx](inv:sphinx:std:doc#*intersphinx), which provides all the cross-documentation linking you see even just on this very page.
Since it normally tries to fetch `objects.inv` files (inventories of site content) from the sites themselves, this too needs to be set up locally.
Thankfully, it's as simple as pointing it to a locally cached file in its [mapping](inv:sphinx#intersphinx_mapping) rather than fetching the files every time the docs are built - they're updated in the project's [CI/CD](project:ci-cd.md) pipeline before building anyway, so they'll always be correct.

### Figures

The next item on the list is the embedded figures - you may notice that they look suspiciously like draw.io diagrams, and that's because they are!
This is detailed more in the next section, but they're built using the tool's command-line interface to SVGs, which can then be referenced directly in the docs like this:

```{image} /generated/example.drawio.svg
:alt: Example draw.io diagram
:class: has-dark-opt
:align: center
```

#### Themeing

If you switch between the light and dark themes, you'll notice that the diagram automatically switches themes too - there's a bit of JavaScript running which automatically switches in the correct image URL based on the currently selected theme, which you can observe if you wish in `_static/js/dark-opt-images.js`.
This is done using JQuery, which as was previously mentioned is installed with a Sphinx extension.
Unfortunately, Sphinx doesn't include images in the final output which aren't referenced directly in the documentation, so normally the dark theme images wouldn't be linked and would therefore be inaccessible - this is fixed using a dirty hack which you can see [here](project:/generated/_figure-index.rst) if you wish.
In the project `conf.py`, there's a hook set up to run before the build starts which iterates through every figure in the `generated` directory (where the figures live) and adds them to an index file, which then ensures that they're therefore included in the output.

## Build Infrastructure

As was previously mentioned, this project is built with Nix, which occasionally brings with it certain challenges (to put it mildly...).
Whilst this is great from a reproducibility standpoint, certain software simply wasn't ever intended to be used in the way that Nix does it.
Python (and therefore Sphinx) and draw.io unfortunately both fall into this category and need special treatment to work properly.

### Python Environment

The first obstacle is Python packages - the credited [blog posts](#credits) which first inspired this toolchain make use of Poetry2Nix, but this is no longer maintained, and the author of the tool recommends using [uv2nix](https://github.com/adisbladis/uv2nix) instead, which is what we have done here.
The [`uv`](https://docs.astral.sh/uv/) tool is a Python package manager which produces a `uv.lock` lockfile containing enough information to satisfy the purity requirements for a Nix build (specifically versions and hashes of **all** dependencies).
If that didn't make much sense, don't worry - it just means that the build is reproducible.
Additionally, if you want to develop directly instead of through Nix, the tool is _extremely_ fast - the speed comparison chart is available on their website for your convenience to see how ludicrous the speed differences are.
Nix then parses the python project file (`pyproject.toml`) along with the lockfile and uses it to build a workspace with `uv2nix` containing all of the Python packages in a `virtualenv`.
This allows repeatable and easy integration of _any_ python module, not just those available in the standard `nixpkgs`, in a much more convenient fashion.

### Draw.io diagrams

% spellchecker:off
% we need to disable the spellchecker for xvfb explanation, and we can't put comments inside the paragraph since it breaks it into multiple paragraphs.
Next up is draw.io.
Whilst, to its credit, it _does_ provide a CLI for interacting with and exporting diagrams, draw.io itself has some... quirks... which make it hard to automate.
The first of these is that it _requires_ a graphical environment to be available to run, even though it isn't using it in the CLI mode (thanks Electron!).
To get around this, the [`drawio-headless`](https://github.com/NixOS/nixpkgs/blob/master/pkgs/applications/graphics/drawio/headless.nix) package was created, which wraps the draw.io call using `xvfb` (**X** **v**irtual **f**rame**b**uffer) and calls it that way.
Unfortunately, if you try and use this package, you'll notice that it doesn't work - it executes, waits for 3 seconds, and then exits, without ever actually running draw.io!
Upon testing, it turns out that the `--auto-display` flag is to blame, and running the command without that flag works perfectly.
To actually generate the diagrams, the script uses `find` to locate all the `.drawio` diagrams in the source directory `docs/figures-source`, which is then passed into GNU `parallel`, which runs the conversion script.
To avoid spinning up a virtual framebuffer for every single `drawio` call, the root `parallel` command is executed through `xvfb-run`, and the X11 environment is then inherited by all the script processes it runs.
Finally, the aforementioned script does one thing - it takes in a source file path and an output directory, and then generates two SVG files from the diagram - one in light theme, and one in dark theme.
% spellchecker:on

### Deployment

As detailed in the [CI/CD Architecture](project:ci-cd.md) document, this entire project uses Nix for everything - and that extends to building in the CI/CD pipeline.
However, that only covers the CI part of CI/CD - deployment needs its own handling.

The built documentation is currently hosted using [GitHub Pages](https://pages.github.com/), and as such requires a backing repository containing the contents which is located [here](https://github.com/ROAR-QUTRC/roar-qutrc.github.io).
:::{note}
This is a "top-level" repository - this means that we can place a `robots.txt` file inside it, and it will be obeyed since it's located under the website root in the final deployment.
Currently it's configured to block all crawlers, thus keeping the site off any search engines (although the repo itself can still be found).
:::
When changes are pushed to this repository, they will automatically be deployed to GitHub Pages using the standard static file deployment workflow.
However, this leaves another challenge - how do we automatically push changes from the source CI/CD pipeline?

The answer to this question is a [deploy key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/managing-deploy-keys#deploy-keys) for the website repo.
Authenticating with a deploy key allows an actor full read/write access to the contents of the repository, which is perfect for this use-case.
Using it with the GitHub Checkout Action leaves the checked out repo in the workflow in a writable state, so we can push changes back upstream.
:::{note}
The key itself is stored using GitHub Secrets and passed as a parameter to the Checkout step.
:::
Finally, to make the deployment, the workflow replaces the contents of the checked out repo with the built static documentation from `nix build .#docs`, then commits and pushes the documentation to the website repo.
Once this runs, the automated deployment is complete, and all going well, the new website is live!

## Nix outputs

This section documents the outputs made available from the project `flake.nix`.

### Build artifacts

Available using `nix build .#name`.

#### `docs`

The final built documentation.
Contains a `html/` directory with the built static site ready to be served in it.

#### `docs.figures`

Built SVGs from the `.drawio` diagrams in `docs/figures-source`.
All file output names are in the format `BASENAME.svg` for the light theme document, and `BASENAME-dark.svg` for the dark theme document.

:::{attention}
Note that `BASENAME` includes the original `.drawio` extension!
:::

For example, an input file `example.drawio` would result in the output files `example.drawio.svg` and `example.drawio-dark.svg`.
In the documentation build process (and when you run the [setup script](#docssetup)), the SVG files are copied into `docs/source/generated` and can be referenced from there.
Additionally, you should add the `has-dark-opt` class so that the theme switcher script can correctly modify the figure - due to transparent backgrounds, light mode figures are not usable in dark mode, and vice versa.
For example, to include the above example figure in a standalone [block](inv:myst#*images_and_figures), the Markdown code is:

````{code-block} markdown
```{image} /generated/example.drawio.svg
:alt: Example draw.io diagram
:class: has-dark-opt
:align: center
```
````

The equivalent _inline_ code (standard markdown image insertion, with extension from MyST-parser for classes) is:

```{code-block} markdown
![Architecture](/generated/example.drawio.svg){.has-dark-opt}
```

:::{note}
References in Sphinx without a leading slash are relative to the current directory!
To reference relative to the project root (in the repo, `docs/source`), you need to make it an _absolute_ path and start it with a slash.
Not doing so makes it relative to the current directory of _this document_ (`docs/source/architecture/`).
:::

#### `docs.compressed`

The built documentation, compressed using `p7zip`.
Contains a single file `docs.7z`.

#### `docs.shell`

Not intended for direct use.
This is the development shell environment for use with `nix develop`.

### Executables

Run using `nix run .#name`.

#### `docs.decompress`

A wrapper script around `p7zip` which provides the arguments to decompress the file built by `docs.compressed` correctly.

##### Arguments

Both arguments are mandatory.

```{option} source
The path to the input file
```

```{option} destination
The path to the destination directory for the extracted files
```

#### `docs.fetch-inventories`

A script which, in order:

1. Fetches the intersphinx inventory files which the project needs
2. Copies them into `docs/source/intersphinx`
   (fetch-inventory-stash)=
3. Attempts to stash your current git index so that it can commit the changes it made (if there are any)
4. Makes a git commit with the current date and time
5. Pops the stash it just made, (hopefully) restoring your git index

```{warning}
[Stashing the index](#fetch-inventory-stash) can fail!
This appears to happen reliably when only committing sections of a file, or in certain cases with untracked files.
If it does, you may be left with a git stash which you may need to pop/drop manually - check with `git stash list` if it outputs an error.
```

```{note}
In certain cases, your index may not be fully restored (ie, files which were staged no longer are) after the script completes.
Double check it before you commit your changes.
```

It's intended for use in the CI/CD pipeline, but you can run it manually if you need to - such as if you're adding a new intersphinx inventory and need to build the docs to test it.

##### Flags

```{option} --no-commit
If present, the script will not automatically commit the changes
```

#### `docs.setup`

A script which needs to be run before running `make html` from a dev shell.
It builds the documentation figures from `docs.figures` and copies them into `docs/source/generated` so that Sphinx can use them.

## Credits

This documentation is loosely based off of Rohit Gaswami's blog posts documenting [setting up](https://rgoswami.me/posts/doc-cpp-dox-sph-exhale/) Sphinx, Breathe, and Exhale, as well as the follow-up detailing [publishing](https://rgoswami.me/posts/pub-doc-cpp-dox-sph-nix/) them with Nix.
