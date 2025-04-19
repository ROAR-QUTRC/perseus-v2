# Systems

This section describes what systems are present on Perseus, what they are and what they do. For information how to develop each of these systems check their corresponding pages in the <project:development-index.md> section.

Fundamentally, the rover is split into two main sub-systems: Hardware, and Software.
The [_software_ architecture](project:/architecture/software.md) lays out how the _code_ interacts with itself and its environment, as well as which bits do what.
The [_hardware_ architecture](project:/architecture/hardware.md) goes over how everything's _physically_ connected and laid out, as well as the electrical wiring.

Whilst those two documents contain the majority of the information you'll need day-to-day, there are also some other moving parts to consider as well:

- The [CI/CD](project:/architecture/ci-cd.md) pipeline is what runs our automated testing and handles deploying this very website
- Speaking of this website, its build process can be a little bit convoluted, so it gets a [document](project:/architecture/documentation.md) too
- If you're writing firmware or a hardware interface that talks on the CAN bus, you'll need to read through what we've titled the [Canifesto](project:/architecture/can-bus.md)
- You've probably noticed by now that we use Nix to manage all our software - this gets complicated quickly and there's only so much that source comments can do, so it's explained [here](project:/architecture/nix.md)

If you're just looking for a high-level overview, this diagram contains the basics of the information flow between modules (as well as some power information):
![Architecture](generated/system_architecture.drawio.svg){.has-dark-opt}

Finally, if you're wondering about a specific payload, those get their own [section](project:/subsystems.md).

```{toctree}
:maxdepth: 1
:titlesonly:
:hidden:
:glob:
systems/*
```
