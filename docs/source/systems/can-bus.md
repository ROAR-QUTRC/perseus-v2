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

CAN Bus networks must have terminating resistors at both ends of the circuit, connecting CANH to CANL through a 120 Ohm resistor (we usually use two 60Ohm and a capacitor in between connecting to GND for reducing noise).
These terminations are both on the Rover Control Board (RCB - large purple PCB in the middle of Perseus). This means that the CAN Bus circuit must start at the RCB, loop around the rover, and end at the RCB.

Any devices connecting to the CAN Bus must have their CANH and CANL wires connected into the DE-9 cable running through Perseus through a **branch**. However, once these branches reach around 30cm, the signal breaks down.
This means that for longer connections like payloads, we must run a **loop** through the subsystem, with each device making a small branch off this loop. The loop starts with the [CAN Bus Daisy Chain PCB](../_static/CAN_Bus_Daisy_Chain.pdf), feeding the main CAN network (CAN_A_L and CAN_A_H) from 'Multi 1 connector' into the 'Main Connector' which goes out to the device.
The device has a branch to these CAN wires, using them to communicate over the CAN network. Then the CAN_A Low and High are connected directly into CAN_B Low and High.
These wires are returned through the same cable to the CAN Bus Daisy Chain, which connects CAN_B_L and CAN_B_H of the 'Main Connector' into CAN_A_L and CAN_A_H of the 'Multi 2 Connector', which connects into the main CAN Bus system, thus completing the loop.

### Packet Construction

Each Packet is made of an address and a data section.

#### Addressing

The CAN protocol Perseus uses is the base CAN Bus protocol with extended addresses (29-bit).
The addresses are split into five sections - System ID (6 bits), Subsystem ID (3 bits), Device ID (4 bits), Group ID (8 bits), and Parameter ID (8 bits).
This way we can have meaningful addresses split up into their purposes (e.g. all drive devices will have the system ID 0x00, while power devices will all have the system ID 0x01).

The System, Subsystem, and Device IDs should specify which device the address points to (e.g. different Smol Brains).
The Group ID should specify which part of the device the address points to (e.g. different sensors/outputs on the same Smol Brain).
The Parameter ID should specify the function the group should perform or the data it is sending (e.g. turn on/off, status).

#### Data

Some packets don't need to send data (e.g. turning on/off)
A packet can hold up to eight bytes of data.
For lengths of data longer than this, the data must be split into multiple packets.

:::{note}
Some packets don't need to send any data (e.g. turning something on/off).
These can be sent like a regular packet without specifying the data.
:::

## Software

The rover's CAN bus library (Hi-CAN) is split into a few different components.
First is the `hi-can` core - this is pure C++, and defines the API for interacting with the CAN bus, how packets and parameters work, assigns addresses, and more.
Note that to allow it to run both on microcontrollers and on Linux systems (thus eliminating code duplication), the core has to be system-agnostic, with target-specific code split into separate libraries.
Next is the raw SocketCAN implementation for Linux systems - `hi-can-raw`.
This is what almost all Linux code needing CAN bus access will be interacting with.
Finally, there's the specialisation for microcontrollers - in this case, the library is also named `hi-can`, but is located in the firmware/components directory.
The header file that firmware should use is `hi-can-twai` in the firmware directory's `hi-can` library.

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

The addresses of each system, subsystem, device, group, and parameter can be found in `software/shared/hi-can/include/hi_can_address.hpp`.

#### Data

Data must be transmitted as bytes. This means that before any data is transmitted, it must be **serialised** - split into a vector of {type}`uint8_t`.
It also needs to be **deserialised** once the data is received - changed back from a vector to the original type (defined in the `software/shared/hi-can/include/hi_can_parameter.hpp` file).

To serialise data, use the {any}`parameters::BidirectionalSerializable::serialize_data()` method of the {class}`parameters::BidirectionalSerializable` class.
This returns the serialised data which can then be assigned to a packet using the {any}`Packet::set_data()` method.

To deserialise data, use the {any}`parameters::BidirectionalSerializable::deserialize_data()` method on the serialisable that you want to store the data.

:::{note}
As described in <project:standards/software.md>, software uses American spelling, so any functions/methods called in software will use the spelling _serialize_ instead of _serialise_.
:::

##### Serialisables

The main serialisable which should be used for all data types is called {class}`parameters::SimpleSerializable`.
This can be used on any struct:

```{code}
typedef SimpleSerializable<_struct_t> serializable_struct_t;
```

Or it can be used on any base type using the {struct}`parameters::wrapped_value_t` struct:

```{code}
typedef SimpleSerializable<wrapped_value_t<int32_t>> serializable_int32_t;
```

#### CAN Interface

A CAN Interface is what allows programs to interface with the CAN network. There are different kinds of CAN Interfaces, all of which inherit from the {class}`FilteredCanInterface` class:

1. The {class}`TwaiInterface` class is used on firmware - it's defined in the hi-can library in the firmware/components directory.
2. The {class}`RawCanInterface` class is used on all Linux machines - it's defined in the `hi-can-raw` library in the software/shared directory.

The {class}`PacketManager` is initialised using a {class}`FilteredCanInterface` (any kind).

The CAN Interface can be set to only receive from specific addresses using **filters**.
Filters can be added or removed using the {any}`FilteredCanInterface::add_filter()` and {any}`FilteredCanInterface::remove_filter()` methods.
When filter/s are applied to the CAN Interface, only a {class}`Packet` with an address matching the filter/s will be allowed through.

The CAN Interface can be used directly to receive or transmit a {class}`Packet` using the {any}`CanInterface::transmit()` and {any}`CanInterface::receive()` methods.

#### Packet Manager

The {class}`PacketManager` class manages scheduled transmission of packets as well as packet reception and timeouts. The {class}`PacketManager` class contains:

1. Callbacks - functions called when a packet is received
2. Transmissions - functions called periodically to transmit data
3. CAN Interface - the interface used by the {class}`PacketManager` to access the CAN network.

The {any}`PacketManager::handle()` method must be called (usually in a loop) to transmit and receive packets using the {class}`PacketManager`.

##### Callbacks

A callback is set using the {any}`PacketManager::set_callback()` method.
It is called using a {struct}`addressing::filter_t` and a {struct}`PacketManager::callback_config_t`.
The {struct}`addressing::filter_t` specifies what addresses the callback applies to.
The {struct}`PacketManager::callback_config_t` specifies a timeout duration as well as callback functions which are called when receiving data, when the timeout occurs, and when data is received after the timeout occurs.

##### Transmissions

A transmission is set using the {any}`PacketManager::set_transmission_config()` method.
It is called using an {struct}`addressing::flagged_address_t` and a {struct}`PacketManager::transmission_config_t`.
The {struct}`addressing::flagged_address_t` specifies what address the transmission should send from.
The {struct}`PacketManager::transmission_config_t` specifies the function that returns the data to transmit, the interval between transmissions, and whether the {class}`Packet` should be transmitted immediately.

Any transmission configs already set in the {class}`PacketManager` can be updated using the {any}`PacketManager::set_transmission_config()` and {any}`PacketManager::set_transmission_interval()` methods.
The transmission configs can be removed using the {any}`PacketManager::remove_transmission()` method.

##### Parameter Groups

The {class}`parameters::ParameterGroup` class can also be used with the {class}`PacketManager`.
A {class}`parameters::ParameterGroup` is defined with a set of callbacks and/or transmissions, which can be added to/removed from a {class}`PacketManager` using the {any}`PacketManager::add_group()` or the {any}`PacketManager::remove_group()` methods.
This allows for the same callbacks and transmissions to be used in multiple places without having to write each callback and transmission directly with the {class}`PacketManager`.
