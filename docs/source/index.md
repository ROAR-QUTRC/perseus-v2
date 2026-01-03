# Home

Welcome! This is the home of the Perseus Rover documentation. It contains (well, it should, if people have been updating it) everything you need to know about building software and electronics for the rover.

If you're new here, you'll probably be wanting to read through the <project:/home/getting-started.md> page, which contains everything you need to know to get yourself set up to start writing code.
Alternatively, if you're instead arms-deep in the rover's guts and wondering where that wire goes, or you're interested in designing electronics to go on-board the rover, check out the [hardware system](project:/systems/hardware-index.md).
Finally, if you're wondering what a specific bit of code does, you probably want look over the [software system](project:/systems/software-index.md), or alternatively there's the auto-generated [code documentation](project:/generated/index.rst).

## Conventions

### File Paths

Unless otherwise specified, file paths are relative to the repository root.
:::{example}
If you checked out the repository at `~/perseus-v2`, `docs/source/index.md` (this file!) would refer to `~/perseus-v2/docs/source/index.md`.
:::

![Perseus](_static/Logo-Complex.png)

% note: elements in this toctree are manually ordered rather than just using a glob

```{toctree}
:maxdepth: 1
:hidden:
:glob:
self
home/getting-started
home/*
systems-index
standards-index
development-index
maintenance-index
challenge-breakdowns-index
tutorials-index
generated/index
```
