# Hi-CAN Tutorials

As the Hi-CAN library can be fairly complex, it has been split into a sequence of tutorials of increasing complexity.
We recommend reading through at least part of <project:/systems/can-bus.md> first, then starting with [](./hi-can/01-hi-can-basic.md).

## Setup

All tutorials in this section are intended to be run on Linux, after you've followed the main <project:/home/getting-started.md> documentation.

Although you _can_ use real CAN hardware for these tutorials, a {term}`VCAN` bus is much more convenient, and won't come with any potential hardware issues.
If you haven't already, run:

```console
nix run .#scripts.vcan-setup
```

This will create the `vcan0` CAN bus, which all examples in this series will use by default.
In the event that you _really_ want to use actual hardware, either replace references to `vcan0` in the code with the actual bus you want to use (eg `can0`), or provide the bus name as an argument to the example programs, which will all connect to what you specify instead.
For example (note that since it is being run using `nix run`, `--` is needed before the list of arguments to be passed to the program being run):

```console
nix run .#pkgs.examples.hi-can.basic-tx -- can0
```

```{important}
All Hi-CAN types are under the [`hi_can`](project:#namespace_hi_can) namespace, although the references in the tutorials may exclude that.
Follow the links for full paths.
```

```{toctree}
:maxdepth: 1
:titlesonly:
:hidden:
:glob:
hi-can/*
```
