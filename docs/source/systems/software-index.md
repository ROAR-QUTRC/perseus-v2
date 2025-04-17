# Software

The software on the rover is split into the core software (mainly ROS2 code), the web control UI, and firmware.
Since the web UI runs on a completely different stack to the rest of the rover code, it just made more sense to separate it out and treat it separately.
Firmware is compiled completely separately to the rest of the stack, so also needs to be split out (although it does share some libraries with the core software).
:::{tip}
If you want to read through project usage, that'll be documented either in the source code (particularly for libraries) and as such be visible in the [generated documentation](project:/generated/index.rst), or in a README file in the project's directory.
:::

```{toctree}
:glob:
:hidden:
software/*
```
