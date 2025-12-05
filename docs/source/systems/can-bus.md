---
hero: The Canifesto
---

# CAN Bus Architecture

On Perseus, most embedded systems communicate over the [CAN bus](https://en.wikipedia.org/wiki/CAN_bus).
To ensure that systems on Perseus work as reliably and with as little hassle as possible, we have created hardware and software standards to which CAN devices should be designed.

## Hardware

CAN transmits data over a twisted wire pair of CAN high (CANH or CAN+) and CAN low (CANL or CAN-).
To make connecting systems easy, almost all subsystems and PCBs use the [DE-9](https://en.wikipedia.org/wiki/D-subminiature) connector (usually called DB9 - as noted in the Wikipedia page, this is a misnomer that's stuck) for attaching to the CAN bus, where the female connector is mounted to the PCB, and the cable has 2 male ends.
In this application, pin 2 is CAN low, and pin 7 CAN high, this being the most [common pinout](https://en.wikipedia.org/wiki/CAN_bus#Layers) for CAN bus on a DE-9 connector (potentially with pin 3 as ground, and pin 9 as power).
When the connectors are used with a return conductor pair (more on that later), pins 1 and 8 are used as return CAN low and high respectively, as they seem to be the least commonly used for other purposes.

Some direct-to-board applications (like connecting a USB-CAN adapter) use a 2-pin JST-XH connector (CANH: Pin 1, CANL: Pin 2), although this is only possible for short branches.

### Loops and Branches

### Packet Construction

#### Addressing

TODO: Explain address bit allocation and priority

## Software

The rover's CAN bus library (Hi-CAN) is split into a few different components.
First is the `hi-can` core - this is pure C++, and defines the API for interacting with the CAN bus, how packets and parameters work, assigns addresses, and more.
Note that to allow it to run both on microcontrollers and on Linux systems (thus eliminating code duplication), the core has to be system-agnostic, with target-specific code split into separate libraries.
Next is the raw SocketCAN implementation for Linux systems - `hi-can-raw`.
This is what almost all Linux code needing CAN bus access will be interacting with.
Finally, there's the specialisation for microcontrollers - in this case, the library is also named `hi-can`, but is located under the firmware directory.

### Architecture

All `hi-can` software is under the {any}`hi_can` namespace, which will not be specified for the sake of brevity.

```{namespace} hi_can

```

#### Addresses

Depending on where in the stack you are, `hi-can` uses one of a few different layers of abstraction to represent CAN packet addresses.
The first and most basic is simply the {type}`addressing::raw_address_t`, representing the underlying 9- or 29-bit long packet ID.
However, actual packets have several flags associated with them needing storage, so the {struct}`addressing::flagged_address_t` struct was created to to serve that need.

As detailed in <project:#addressing>, packet IDs are split into several different sub-fields.
Although much of the library interacts directly with the 29-bit IDs of the CAN bus using either the raw or flagged address types, the {struct}`addressing::standard_address_t` struct has also been created to allow for easily creating or modifying addresses in a much more human-friendly fashion.
Both the {struct}`addressing::flagged_address_t` and {struct}`addressing::standard_address_t` inherit from {struct}`addressing::structured_address_t` so that they are guaranteed to easily convert to a standard {type}`addressing::raw_address_t` with a simple cast.
This means that any time you need to provide a raw address, you can build it with an easier to use struct and then convert it when needed.
