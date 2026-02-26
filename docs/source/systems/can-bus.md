---
hero: The Canifesto
---

# CAN Bus

On Perseus, most embedded systems communicate over the [CAN bus](https://en.wikipedia.org/wiki/CAN_bus).
To ensure that systems on Perseus work as reliably and with as little hassle as possible, we have created hardware and software standards to which CAN devices should be designed.

Depending on what you're doing, you may only need to read specific sections of this document.
If you're doing hardware design or physically connecting the CAN bus, the [](#hardware) section will obviously be most relevant to you.
Alternatively, if you're writing software, you'll probably find the [](#hi-can-usage) section most useful, with [](#hi-can-architecture) for necessary background information and terminology.
Finally, reading through the whole [](#protocol) section will probably be helpful when debugging issues with the bus, and will likely improve your understanding of the Hi-CAN library's usage.

## Hardware

CAN transmits data over a twisted wire pair of CAN high and CAN low [^can-hl].
To make connecting systems easy, almost all subsystems and PCBs use the [DE-9](https://en.wikipedia.org/wiki/D-subminiature) connector (usually called DB9 - as noted in the Wikipedia page, this is a misnomer that's stuck) for connecting to the CAN bus, where the female connector is mounted to the PCB, and the cable has 2 male ends.
In this application, pin 2 is CAN low, and pin 7 CAN high, this being the most [common pinout](https://en.wikipedia.org/wiki/CAN_bus#Layers) for CAN bus on a DE-9 connector (potentially with pin 3 as ground, and pin 9 as power).
When the connectors are used with a return conductor pair (more on that later), pins 1 and 8 are used as return CAN low and high respectively, as they seem to be the least commonly used for other purposes.

[^can-hl]: Typically some variation of `CAN_H`/`CAN_L`, `CANH`/`CANL`, or `CAN+`/`CAN-` in schematics.

```{figure} /generated/can-bus.drawio.svg
:alt: CAN bus connectors
:align: center
:class: has-dark-opt

DE-9 CAN bus pinout from outside face
```

Some direct-to-board applications, like connecting a USB-CAN adapter, use a 2- or 3-pin JST-XH connector [^jst-pins], although this is only possible for short {term}`branches <branch>`.

[^jst-pins]: For 3-pin JST connectors: Pin 1 - GND, pin 2 - CAN High, pin 3 - CAN Low. For 2-pin connectors: Pin 1 - CAN High, pin 2 - CAN Low.

(can-loops-branches)=

### Loops and Branches

All devices attached to the CAN bus require at least 3 connections: CAN high, CAN low, and ground.
The pair of CAN high/low wires that attach to the bus are called a _branch_, and must be less than 30cm long, otherwise signal reflections will cause interference and potentially stop the bus from working properly.
However, for longer connections like payloads, this means the bus must _loop_ out to the device, which can then attach via a short branch, and back.
To save space and wiring, the conductor pairs for both ends of the loop are typically run through the same DE-9 connector.
This is where the <project:/hardware/can-daisy-chain.md> comes in.
Normally, only the "standard" CAN bus pins 2 and 7 (the {term}`"A" pair`) are used, but when the bus needs to make a loop, pins 1 and 8 (the {term}`"B" pair`) make the second connection, also called the "return" connection.
However, wiring the bus this way means that the loop must **always** be completed, otherwise the bus will be split in two.
When designing CAN bus {term}`node`s, the easiest way to do this is usually to put a standard DE-9 connector on-board the PCB, and connect its A and B pins together there.
Since the bus is continuous up until the connector, thanks to the loopback being on the PCB, the PCB connection can then make use of the full 30cm of branch length.
If the node is disconnected, then a {term}`loopback connector` will need to be fitted in its place to complete the loop.

### Termination

CAN bus networks require 120 Ohm terminating resistors at both ends of the bus, which CAN high to CAN low.
On Perseus, the termination on both ends of the bus is on-board the {term}`RCB`, and uses _split termination_.
Rather than a single 120 Ohm resistor, split termination uses two 60 Ohm resistors [^60-ohms] with a capacitor in the middle, as this filters out some noise - see [^split-termination] for more information.
Since both terminations are on the RCB, the CAN bus must both start and end there, making a full loop around the rover.

[^60-ohms]: On the RCB, these have actually been substituted with **much** more readily available _62_ Ohm resistors for cost and availability.

[^split-termination]: For more information on split termination, see section 7.2.2 in [this datasheet](https://www.ti.com/document-viewer/TCAN332/datasheet#GUID-BE537D5C-327A-49E5-8C24-D63E50312EEB/TITLE-SLLSEQ7SLLSEQ72828).

### Connectivity

Every device on the CAN bus requires what is called a _controller_ to actually implement the protocol.
Controllers then have `CAN_TX` and `CAN_RX` lines, for transmitting and receiving data respectively [^can-rx-tx].
However, to actually connect to the bus, a _transceiver_ is also needed, which converts the TX/RX lines to and from bus voltage levels.
Many microcontrollers, such as the ESP32-S3 that powers the {term}`SBB`s, have controllers built in.
However, almost no devices include a transceiver as well, which must usually be provided externally.
In the case of the SBBs, it is built in to the PCB.

Additionally, to connect to a CAN bus on most Linux devices, you will need a USB to CAN adapter - although the Jetson Orin Nano is an exception, with its built-in CAN controller.
Note that there are two main types of adapter - SocketCAN, and SLCAN (Serial Line CAN).
SocketCAN devices will be immediately available to configure when plugged in, but SLCAN devices will first need to be set up with {manpage}`slcand(1)` before they can be configured and enabled.
Unfortunately, despite the added complexity, SLCAN devices tend to be much more common than SocketCAN ones.

[^can-rx-tx]: These lines are normally high by default, and pulled _low_ when a 1 is being transmitted.

## Protocol

### Signalling

CAN is a _differential_ protocol, meaning that a one or a zero being transmitted is determined by the _difference_ between two lines, rather than the absolute voltage relative to ground.
Aside from power, nodes only need a ground connection to ensure that the absolute difference between between CANH or CANL and ground doesn't become high enough to damage components.
To transmit a 0, called the _dominant state_, CAN low is pulled to 0V, and CAN high is pulled to 5V.
This is called the bus being _asserted_.
However, to transmit a 1, called the _recessive state_, the bus is simply released, and the termination resistors pull the lines to an equal voltage - typically about 2.5V.
This means that a 0 on the bus is given by CANH $>$ CANL, and a 1 by CANH $\leq$ CANL, with some tolerance.

### Frames

CAN transmits data in _frames_, which come in several different types.
The first, and most common, type is a _data_ or _message_ frame, which we often call a _packet_.

Every CAN bus data frame consists of an address (or ID) and some flags, with up to 8 bytes of data.
This address can be either 11 or 29 bits long, depending on the first of these flags, called the IDE (Identifier Extension Bit) flag.
If the IDE flag is a 0 (dominant), the frame uses an 11-bit address, called {term}`Standard Frame Format`.
This frame format is is also called CAN 2.0A.
However, if it is a 1, the frame instead uses a 29-bit address, called the {term}`Extended Frame Format`, or a CAN 2.0B frame.

```{note} Data length
Data frames don't actually have to contain any data!
They can be transmitted with 0 bytes of data.
However, this is uncommon and usually not very useful.
```

The next type of frame is a _remote frame_, and is very similar to a data frame.
It contains the same address section and set of flags, but has the RTR, or Remote Transmission Request, flag set.
When that flag is set, the frame can contain no data, and is instead _requesting_ data be transmitted from a remote node.

Both remote and data frames have an _acknowledge_ (ACK) slot, which the transmitting node will leave recessive (1).
If any other nodes on the bus have correctly received the frame, they will pull the ACK slot to 0 (dominant), indicating successful reception.
Alternatively, if the slot is left recessive, the transmitting node knows that there was an error in transmission (since no other devices acknowledged it).
The transmitter can choose to handle this in different ways, but will almost always just attempt to re-transmit the frame.

```{warning}
When developing CAN bus nodes, make sure you either disable requiring acknowledgement [^no-ack], or have at least one other device on the bus.
If you do not, the device under test will rapidly enter an error state due to none of its frames being acknowledged, which would typically mean a hardware failure.
```

[^no-ack]: On Linux, this means setting [`presume-ack`](https://www.kernel.org/doc/html/latest/networking/can.html#netlink-interface-to-set-get-devices-properties) to `on`, and in firmware, setting the `HI_CAN_NO_ACK` `menuconfig` option.

Finally, the last common frame type is an _error frame_, and is detailed in the [](#errors) section.

### Errors

Unless you're debugging the CAN bus or writing low-level code that interacts with it, you can probably skip this section.

#### Error Detection and Flags

If a node detects an error during frame transmission or reception, it will transmit an error active or error passive flag.
To ensure that all nodes stay synchronised, a single bit of the opposite polarity is inserted into the frame after 5 identical bits, which is called _bit stuffing_.
Error flags exploit this, as they are a deliberate sequence of _6_ identical bits - dominant for an error active flag, and recessive for an error passive flag.
This has the effect of simultaneously destroying the frame with an "artificial" error and signalling to all devices that it was incorrect - making it an error frame.
An error flag will be raised by every node that detects an error, regardless of whether it was the original transmitter or not, as otherwise the transmitter might not know that there was a problem.

In practice, this looks like at least one node detecting an error, and then transmitting an error flag - this is called the _primary_ error flag.
This error flag will then force any _remaining_ nodes to detect a bit stuffing error, and they will then raise their _own_ error flags, called _secondary_ error flags.
As such, an error bit sequence can last anywhere from 6-12 bits, depending on when each node detects the fault.

#### Error Handling and Counters

The CAN specification dictates that nodes must implement two different counters: The Transmit Error Counter (TEC), and Receive Error Counter (REC).
As the counters increase, nodes must then take themselves into error states.
By default, every node is in the _Error Active_ state, which is just normal operation.
However, if either counter exceeds 127, the node will enter the _Error Passive_ state, detailed more below.
If the counters keep increasing and either passes 255, the node must then enter the _Bus Off_ state, where it fully disconnects itself from the bus and can no longer transmit data, acknowledgements, or error flags.
For ESP32-based firmware, this is when _bus recovery_ can be (and is) initiated.
Alternatively, for Linux, the `restart-ms` option will re-start the bus after a set delay, although this is disabled by default.
Many CAN devices (including Linux and ESP32's) also implement an Error Warning state when a counter is above 95 but not yet in the Error Passive state so applications can take preemptive action.
In the Error Passive state, nodes can still operate mostly normally, but will only transmit Error Passive flags instead of Error Active, and must wait an additional delay after the end of each frame before beginning transmission to give normally operating nodes precedence.

When a _transmitting_ node detects an error, it will immediately raise a primary error flag (which can be either active _or_ passive), and increment its TEC by 8.
Alternatively, if a _receiver_ raises the primary error flag, it will increment its _REC_ by 8.
However, if a receiver instead raises the _secondary_ error flag, it increments its REC by only 1.
Finally, each time a frame is successfully transmitted or received, the TEC and REC respectively are decremented by 1.
Note that since an Error Passive flag uses _recessive_ bits, it will be "invisible" unless it's actively transmitting.
This means that only transmitting nodes can raise error passive flags, preventing faulty receiving nodes from destroying valid frames once they enter the Error Passive state.

### Arbitration

One of the features of CAN is called _arbitration_, and automatically determines message priority.
If multiple nodes start transmitting at the same time, arbitration ensures that one of them will automatically yield the bus to the other after _losing arbitration_.
The start of a frame, including the ID and flags, comprises the _arbitration phase_ of the message.
During this time, each transmitting node is monitoring the bus.
Since the bus has to be released to a recessive state (1), a node releasing the bus to transmit a 1 at the same time as another node asserts the bus to a 0 will detect this and yield the bus.
The first consequence of this is that loss of arbitration can only be detected by nodes with a 1 instead of a 0 in the arbitration phase.
This then means that frames with a lower ID number will **always take priority** over ones with a higher ID.
Additionally, since the extended ID flag is asserted for {term}`SFF` frames, 11-bit frames will always take precedence over 29-bit extended frames.
The second consequence is since the arbitration phase only lasts for the frame "header", if two nodes start transmitting frames with the same ID, it **will cause an error** if their data differs!

```{error}
This is why ensuring that each device has its own address is critical!
If you don't, nodes may transmit at the same time and fail to negotiate arbitration, causing errors.
```

### Hi-CAN

Since the CAN specification does not mandate anything about its actual use, several different protocols have been created for various uses.
In the case of Perseus, we have written a relatively simple protocol called Hi-CAN---short for hierarchical CAN---as well as a library to implement it.
This section details its design and use.
In the event that the implementation in code deliberately differs from the description in this document, the code should be considered "canonical" and the documentation updated.
Bugs, of course, are an exception.

#### Parameters

Hi-CAN was designed primarily to be simple to implement and understand, but also flexible enough to serve a variety of use-cases with minimal hassle.
To meet this, each device implementing Hi-CAN exposes a set of what are called _parameters_.
Each parameter is a piece of data associated with a single CAN ID.
Parameters should be no more than 8 bytes long, so that they fit in a single CAN data frame (hence, a single CAN ID), and contain a single control input or piece of information.
A piece of information is usually either a status or sensor reading, and may contain multiple related _fields_.
For example, a device status parameter might have uptime, CPU load, and status code fields within it.
Alternatively, a control input might be something like desired motor speed, power bus control, or enabling/disabling a sensor.

```{warning}
Although it may make sense to combine multiple pieces of information into a status parameter, this should only be done for control interfaces with a _very_ good reason.
Otherwise, you're forced to update everything in that parameter at once, even if you only want to change a single thing.
However, this behaviour may be wanted in some cases.
```

A set of logically related parameters are then combined into a _parameter group_, which could typically be described as one "part" or "function" of a device.
It may also often make sense for a device to provide the same parameter group multiple times under different IDs.
For example, a board controlling multiple motors should usually implement a single, standard, motor control parameter group for each motor.
That group might then contain speed and current limit control interfaces, as well as current draw and motor controller status parameters.
Alternatively, if the board has multiple sensors, each sensor might get a parameter group.
In a concrete example, the {term}`RCB` implements the same parameter group for all 4 of its buses, which each provide the same set of control and feedback interfaces.

#### Address Construction

As previously mentioned, frames contain an either 11- or 29-bit long ID, or address.
Perseus only uses extended frames, with Hi-CAN splitting addresses into 5 sections.
In order, those sections are the:

- System ID (6 bits, maximum value 63 or `0x3F`)
- Subsystem ID (3 bits, maximum value 7 or `0x07`)
- Device ID (4 bits, maximum value 15 or `0x0F`)
- Parameter Group ID, or just Group ID (8 bits, maximum value 255 or `0xFF`)
- Parameter ID (8 bits, maximum value 255 or `0xFF`)

Which, in binary, breaks the address up like so [^id-breakdown]: [000AAAAA][ABBBCCCC][DDDDDDDD][EEEEEEEE]

[^id-breakdown]: A: System, B: Subsystem, C: Device, D: Group, E: Parameter. The square brackets group group by byte.

Or, in hexadecimal (note that the subsystem ID, being only 3 bits, shares its hex digit with the system ID): [AA][BC][DD][EE]

Some examples of what the full CAN frame ID looks like in various scenarios are provided below - note the system ID's overlap onto the third digit:

| Scenario                              | ID           |
| ------------------------------------- | ------------ |
| Maximum EFF CAN ID [^id-example-1]    | `0x1FFFFFFF` |
| Maximum System ID                     | `0x1F800000` |
| Maximum Subsystem ID                  | `0x00700000` |
| Maximum Device ID                     | `0x000F0000` |
| Maximum Group ID                      | `0x0000FF00` |
| Maximum Parameter ID                  | `0x000000FF` |
| All IDs 1                             | `0x00910101` |
| All IDs 2                             | `0x01220202` |
| All IDs `0x0F` (`0x07` for subsystem) | `0x07FF0F0F` |

[^id-example-1]: Note that due to the CAN ID being 29 bits long, the maximum value for the first hex digit is only ever 1.

Thanks to CAN [arbitration](#arbitration), this gives the address a _hierarchical_ set of priorities from the various IDs, with the system ID being the most important - hence the name Hi-CAN.
The combination of system, subsystem, and device IDs together identify any physical device on the bus.
When a frame has an ID for a specific device, that means it is either being transmitted _to_ or _by_ that device.
Then, the group and parameter IDs actually select a unique parameter that the device provides.

#### Broadcasting

Devices implementing Hi-CAN should typically be _broadcasting_ everything that isn't a control interface on a fixed interval.
Additionally, when a device receives a remote frame with an address matching any of its parameters, it should immediately transmit that parameter with its latest information.
This allows for any node requiring information more frequently to simply request it using remote frames, and allows nodes to query the current state of control interfaces.

Some devices may also implement control interfaces that must be continuously updated.
These are usually for systems that could become unsafe in the event of control loss, like motors or heaters.
In this case, those interfaces must receive new data within a certain interval of the last update, or they will revert to a fail-safe state.
This behaviour is detailed more in the [](#software) section.

#### Non-compliant Hardware

Of course, not all off-the-shelf devices can be made Hi-CAN compliant.
One example is the {term}`VESC`s used to drive Perseus' motors, which use IDs in the fixed format `0x0000AABB` (A: command ID, B: VESC ID).
Fortunately, in most cases, between configuring the devices and reserving specific system/subsystem IDs, even non-compliant devices can be shoehorned into the Hi-CAN ecosystem.

In the case of VESCs, the solution was fairly easy:

1. Assign system 0 (drive) and subsystem 0 (VESC) to VESCs, thus reserving all frame IDs starting with `0x000`.
   This ensures that no other Hi-CAN devices will transmit packets that could be mistaken for VESC commands.
2. Provide a VESC-specific address builder (a specialised {type}`address_t<hi_can::addressing::drive::vesc::address_t>`), taking care of the ID's differing layout.

## Software

The library implementing Hi-CAN, creatively also named `hi-can`, is split into a few different components.
First is main `hi-can` library - this is pure C++ for portability, and defines the API for interacting with the CAN bus, how packets and parameters work, assigns addresses, and more.
To eliminate code duplication and ensure that the same library core runs on all systems (both Linux and embedded), code in `hi-can` has to be system-agnostic, with all target-specific code split into separate libraries that depend on `hi-can` called _implementations_.

### Hi-CAN Architecture

Note that all Hi-CAN libraries are under the [`hi_can`](project:#namespace_hi_can) namespace, which will not be specified for the sake of brevity.
The main Hi-CAN library is split into 3 sections:

- Core: Abstraction layers, frame construction, handling scheduled frame transmission/reception, managing parameter groups
- [`hi_can::addressing`](project:#namespace_hi_can__addressing): CAN address construction, ID assignments
- [`hi_can::parameters`](project:#namespace_hi_can__parameters): Parameter construction, serialisation and deserialisation, and defining parameter groups

% set the namespace for code snippets

```{namespace} hi_can

```

#### Implementations

The first implementation is the raw SocketCAN implementation for Linux systems - `hi-can-raw`.
This is what almost all Linux code needing CAN bus access will be interacting with.
To use it, projects need to configure their CMake build process to link to `hi-can-raw`, then include the main header file:

```cpp
#include <hi_can_raw.hpp>
```

Next is the specialisation for microcontrollers - in this case, the library is also named `hi-can`, but is located in the [`firmware/components`](project:#dir_firmware_components) directory.
As before, to use the library, the project must first be configured to link to the library, and then include the implementation's header file:

```cpp
#include <hi_can_twai.hpp>
```

Details on setting projects up to link to the implementations can be found under <project:/development-index.md>.

#### Core

At the core of the Hi-CAN library is the {expr}`CanInterface` class.
Although it doesn't do much on its own, being simply an interface for implementations to complete, it defines the library's API for directly interacting with the CAN bus.
Its primary jobs are simply to provide a way to transmit and receive CAN frames - see its documentation for more information.
However, most applications only care about a small fraction of the data on the bus, hence the {expr}`FilteredCanInterface` interface which only receives frames matching its filters (more on that later).
On Linux, the kernel should handle filtering after socket configuration, and in embedded applications, it should usually be handled by hardware.
However, in the few cases where filters need to be implemented on the application side, the {expr}`SoftwareFilteredCanInterface` takes an un-filtered CAN interface and wraps it to provide filtering.

To actually _represent_ CAN frames, the {expr}`Packet` class bundles together data, the frame ID, frame flags, and methods for conveniently setting/getting the data.
This is what the {expr}`CanInterface` class actually deals with user-side.
However, this still leaves the burden of handling interval transmissions or fail-safe parameters, for which the {expr}`PacketManager` class was written.
For data transmission, it handles interval (scheduled) parameters, since single-shot frames can be transmitted with less hassle directly through the {expr}`CanInterface`.
Its primary use-case, however, is data _reception_.
When properly configured, the {expr}`PacketManager` will automatically handle calling callbacks on data reception, parameter timeout, and recovery if data is received after a timeout.
As usual, check its documentation for the full API, or the library [usage](#hi-can-usage) section for the fundamentals.

```{important}
All system-specific addressing and parameter types and variables are defined under their own namespaces.
For example, the list of primary compute device IDs is defined in {enum}`addressing::compute::primary::device`.
```

#### Addressing

Depending on where in the stack you are, Hi-CAN uses one of a few different ways to represent CAN addresses.
The first and most basic is simply the {expr}`addressing::raw_address_t`, representing the underlying 11- or 29-bit long frame ID.
However, actual frames also have several flags associated with them needing storage, so the {expr}`addressing::flagged_address_t` struct was created to to serve that need.

```{namespace-push} addressing::flagged_address_t

```

Flagged addresses have 3 additional members: {expr}`is_extended`, {expr}`is_rtr`, and {expr}`is_error`.
As one would expect, {expr}`is_extended` determines whether the ID is in standard (11-bit) or extended (29-bit) form, and for Hi-CAN usage, should always be `true`.
The next flag, {expr}`is_rtr` is exactly what it looks like: Is the frame a remote (RTR) frame?
Finally, {expr}`is_error` is set if the frame had an error during reception.
Note that it cannot be set when _transmitting_ a frame, as this would be nonsensical.

```{namespace-pop}

```

As detailed in the [Hi-CAN protocol](#address-construction) section, frame IDs are split into several different sub-fields.
Although much of the library interacts directly with frame IDs in either raw or flagged addresses, the {expr}`addressing::standard_address_t` struct has also been created to allow for easily creating or modifying Hi-CAN addresses in a much more human-friendly fashion.
With multiple ways of constructing addresses, the {expr}`addressing::structured_address_t` interface type was created, which guarantees easy conversion to a standard {expr}`addressing::raw_address_t` with a simple cast.
As such, both the {expr}`addressing::flagged_address_t` and {expr}`addressing::standard_address_t` inherit from it, as should any other ways of constructing addresses.
This means that any time the library needs a raw address, it can be built with one of the much more convenient structs instead.

All IDs (system, subsystem, device, group, and parameter) are defined in [{file}`/software/shared/hi-can/include/hi_can_address.hpp`](project:#file_software_shared_hi-can_include_hi_can_address.hpp).

##### Filters

{expr}`addressing::filter_t`'s can be applied to a {expr}`FilteredCanInterface` to allow it to only receive from addresses that match any of the filters.
The {expr}`addressing::filter_t::address` member determines the actual _address_ to match---unless the mask is changed, the filter will only allow that single address.
Next is the {expr}`addressing::filter_t::mask`, which determines which bits in the address to match.
By default, it will match the entire address.
However, if any bits in the mask are cleared, they won't be compared during matching.

```{example}
If the filter's mask is set to `0x0000FFFF` and address to `0x12345678`, it would match `0x12345678` and `0x00345678`, but not `0x12345600`, since it's only comparing the last 16 bits (4 digits).
```

The filter can also be set to match RTR and error flags, with the {expr}`addressing::filter_t::should_match_rtr` and {expr}`addressing::filter_t::should_match_error` flags respectively.

Filters can be _added_ using {expr}`FilteredCanInterface::add_filter()`, and removed with {expr}`FilteredCanInterface::remove_filter()`.
If the interface has _no_ filters applied (either because it's just been created or all filters were removed), it will allow all IDs.

#### Parameters

Everything parameters is located in {file}`/software/shared/hi-can/include/hi_can_parameter.hpp` and its corresponding source file.
To actually _transmit_ data over the bus, it must be _{term}`serialised <serialisation>`_ into a sequence of bytes---that is, an array of {expr}`uint8_t`'s, represented as a {expr}`std::vector<uint8_t>`.
Once it's been received, the byte sequence must then be _{term}`deserialised <deserialisation>`_ back into a standard data type.
Hi-CAN defines the {expr}`parameters::Serializable` and {expr}`parameters::Deserializable` classes for a standard API for handling (de)serialisation, as well as the {expr}`parameters::BidirectionalSerializable` class, which implements both of them.
To serialise data, use the {expr}`parameters::Serializable::serialize_data()`, which returns a buffer that can then be assigned to a frame with {expr}`Packet::set_data()` method.
Alternatively, to deserialise data back into a useful type, use {expr}`parameters::Deserializable::deserialize_data()` instead.
As previously mentioned, almost all parameters should implement the {expr}`parameters::BidirectionalSerializable` for bidirectional transformations.

```{important}
As described in <project:/standards/software/general.md>, software uses American spelling, so any functions/method definitions should use the American _serialize_ instead of _serialise_.
```

Although all systems currently running on Perseus use {term}`little-endian` {term}`byte order`, allowing standard integer types like the {expr}`uint32_t` to be directly copied in and out of buffers, a system's byte order is not always guaranteed.
Therefore, wherever possible, Hi-CAN data should explicitly convert data into {term}`network order` ({term}`big-endian`), so that the byte order is well defined.
This also has the added benefit that the serialised data is easier for humans to interpret.
In code, this usually means using functions like {cpp-inline}`htonl()` (host to network, {expr}`long`) or {cpp-inline}`ntohs()` (network to host, {expr}`short`) during serialisation and deserialisation respectively to guarantee the correct byte order.
However, if transmitting data in the host's byte order, or if the order is guaranteed (like in a packed `struct`), data can be copied directly in and out of the buffers.
This is implemented by the {expr}`parameters::SimpleSerializable` class, which simply wraps the provided type to provide serialisation.
For base types such as the {expr}`int32_t` which cannot be inherited from, the {expr}`parameters::wrapped_value_t` wraps them in a struct so they can be passed into {expr}`parameters::SimpleSerializable` and the like.

````{warning}
If you're serialising a `struct`, make sure it's properly packed!
By default, members are aligned to the system's memory word width---in modern computers, typically 64 bits, or for the ESP32, 32 bits.
This means that any types that take up fewer bits will be "expanded" to the full width.
To disable this behaviour, use the `#pragma pack` directive, usually as follows:

```cpp
#pragma pack(push, 1)
// define the structs here
#pragma pack(pop)
```

Using `push` and `pop` means the changes will only be applied between the two lines, so the rest of your file will remain unaffected.
````

Additionally, although most systems store {expr}`float`s and {expr}`double`s using the IEEE 754 standard, this is nowhere _near_ guaranteed (especially on embedded architectures), and as such copying floating-point numbers directly to/from buffers is **strongly** discouraged.
The recommended alternative is to use _fixed-point_ math and transmit data over the network as an integer with a multiplier.
Rather than transmitting the value `123.456` as a `float`, this means first multiplying by a multiplier (say 1000x), then sending an `int32_t`: `123456`.
This functionality is provided by the {expr}`parameters::scaled_int32_t` and {expr}`parameters::scaled_int16_t` types.

Most of the time, parameters are provided as part of a group, which must implement the {expr}`parameters::ParameterGroup` interface, like for example the {expr}`parameters::drive::vesc::VescParameterGroup`.
Parameter groups are implemented as follows:

1. Create internal private members to store the relevant parameters.
2. Create getters for the parameters, usually as references.
3. In the parameter group constructor, do one or more of:
   1. Add entries to {expr}`parameters::ParameterGroup::_callbacks`: {expr}`PacketManager` callback configurations, for parameters that receive data.
   2. Add entries to {expr}`parameters::ParameterGroup::_transmissions`: {expr}`PacketManager` transmission configurations, for interval transmission parameters.
   3. Override {expr}`parameters::ParameterGroup::get_startup_transmissions()` with a list of frames to transmit upon the group's initialisation.

```{note}
The {expr}`PacketManager` class usage is detailed in the [](#hi-can-usage) section.
```

Parameter groups will typically take an {expr}`addressing::standard_address_t` from which the device address is extracted, and some form of `enum` or group number which determines the group ID.
Since the actual parameter IDs are fixed, the parameter group can then construct the full addresses for all of its internal parameters from those two values.

### Hi-CAN Usage

This section details the practical usage of the Hi-CAN library in actual code.

#### CAN Interface

As previously mentioned, programs require a CAN interface instance to interact with the CAN bus.
Currently, there are two available options provided by the library implementations:

1. The {expr}`RawCanInterface` class for Linux. Defined in the `hi-can-raw` under {file}`/software/shared`. Based on raw [SocketCAN](https://docs.kernel.org/networking/can.html).
2. The {expr}`TwaiInterface` class for firmware. Defined in the `hi-can` library under {file}`/firmware/components`. Based on the ESP32's [{term}`TWAI`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/twai.html) library.

Although each implementation implements the same API, they all come with their own quirks, so you should read each one's documentation thoroughly.
To start using Hi-CAN, you must instantiate one of these options.
The interface can now be used directly to receive or transmit a {expr}`Packet` with the {expr}`CanInterface::transmit()`[^raw-can-tx] and {expr}`CanInterface::receive()` methods.

[^raw-can-tx]: One example of the aforementioned "quirks" is if a Linux CAN interface is connected to `any` instead of an actual can bus like `can0`. In this case, it can _receive_ from all buses, but not _transmit_ to any of them unless the exact bus is specified.

#### Packet Manager

```{namespace-push} PacketManager

```

Directly handling packet transmission and reception is a lot of repetitive code that can be easily shared.
As such, the {expr}`PacketManager` class does exactly that, managing scheduled transmission of frames as well as parameter reception and timeouts.

```{warning}
When constructed, the {expr}`PacketManager` takes over receiving frames from the {expr}`FilteredCanInterface` passed into it.
You **must not** override the interface's receive callback, otherwise the {expr}`PacketManager` can no longer receive data!
However, you can still transmit frames directly with the interface.
```

The class is based around two main features: _callbacks_ and _transmissions_, each of which has its own section below.
These are handled by the {expr}`handle_receive()` and {expr}`handle_transmit()` methods respectively, or usually {expr}`handle()` instead which calls both of them.
Either both receive and transmit, or the main handler must be called regularly for the class to operate.
Usually, this means calling it in your program's main loop or on a high-frequency (typically less than 10ms interval) timer.

##### Callbacks

```{namespace-push} callback_config_t

```

In the {expr}`PacketManager`, callbacks are functions that get called when a frame is received.
A callback is set using the {expr}`set_callback()` method, which requires both a {expr}`addressing::filter_t` and a {expr}`PacketManager::callback_config_t`.
The filter specifies _what_ addresses should be handled, and the configuration specifies _how_ to handle them.
In the configuration, there is the main {expr}`data_callback` function, which gets called any time a matching frame is received, and several other fields which specify how to handle data timeouts.
If a timeout duration is specified with {expr}`timeout`, data is expected spaced {expr}`timeout` apart, whatever that duration happens to be.
In Hi-CAN, 2 frames may be missed before the parameter is considered timed out, at which point the {expr}`timeout_callback` function will be called.
This means that if the timeout is 1 second, the parameter will time out after 3 seconds (1 second of expected delay, plus 2 seconds for missed frames).
Finally, if data is received for the first time after a timeout, the {expr}`timeout_recovery_callback` function is called after the data callback.
To retrieve a pre-existing callback configuration, or remove an unneeded one, you can use the {expr}`get_callback()` and {expr}`remove_callback()` functions, respectively.
If needed, callbacks can also be updated with {expr}`set_callback()`.

```{namespace-pop}

```

##### Transmissions

```{namespace-push} transmission_config_t

```

Transmission configurations are set with {expr}`set_transmission_config()`, and are how the {expr}`PacketManager` handles sending data on a schedule.
Each transmission configuration, stored in the {expr}`PacketManager::transmission_config_t` struct, is associated with a standard {expr}`addressing::flagged_address_t`.
The transmitted frames will all be sent with the provided address, and data provided by the {expr}`generator` function.
Using a function rather than a fixed data buffer makes implementing parameters much easier, since the data generator can just be the parameter's serialisation function.
The {expr}`transmission_config_t` also has two additional members: {expr}`interval`, how often the data should be transmitted, and {expr}`should_transmit_immediately`, whether the data should be transmitted immediately after storing the configuration instead of waiting for a transmit call.
Analogous to callbacks, previously set transmission configurations can be updated using the {expr}`set_transmission_config()` and {expr}`set_transmission_interval()` methods, or removed with {expr}`remove_transmission()`.

```{namespace-pop}

```

##### Parameter Groups

As mentioned previously, the {expr}`parameters::ParameterGroup` class is a conveniently grouped set of callback and transmission configurations to implement a set of parameters.
A parameter group can then be added to or removed from a {expr}`PacketManager` with the {expr}`PacketManager::add_group()` or {expr}`PacketManager::remove_group()` methods.
This allows for the same configurations to be reused in multiple places without having to provide each set of configurations directly to the {expr}`PacketManager`.

```{namespace-pop}

```

### Working Examples

Working examples of how to use the Hi-CAN library are provided in each of its [tutorials](project:/tutorials/hi-can-index.md).

## Terminology

```{glossary}
TWAI
  Two-Wire Automotive Interface. What Espressif call their implementation of CAN for legal reasons.

CAN
  Short for Controller Area Network.

CAN bus
  The physical/logical bus to which devices are connected to construct the network.

Hi-CAN
  Short for Hierarchical CAN. The name of the protocol used in Perseus which dictates how devices communicate on the CAN bus. See [](#hi-can) for more information.

VCAN
  Virtual CAN bus on Linux. Very useful for testing CAN software.

Node
  A device on the {term}`CAN bus`.

Branch
  A short conductor pair (CAN high, CAN low) coming off the {term}`CAN bus` to connect a device. Must be shorter than 30cm.

Loop
  A loop of the {term}`CAN bus` (going out and returning) run "outside" its normal path to a {term}`node` which would otherwise require a long branch.

A pair
"A" pair
  The conductor/pin pair that CAN high and low are normally connected to. If unspecified, the conductors will be an A pair. Typically {literal}`CAN_A_L` and {literal}`CAN_A_H` if specified.

B pair
"B" pair
  Secondary conductor pair to allow for a loop's return path to be in the same cable. Typically denoted as {literal}`CAN_B_L` and {literal}`CAN_B_H`.

Loopback
  Connecting the A and B conductor pairs together to complete a {term}`loop`.

Loop connector
  A connector using both the A and B pairs. Must be attached to a loop which connects the A and B pairs together in order to complete the bus.

CAN jumper
Loopback connector
  A connector which internally forms a loopback.

SFF
Standard Frame Format
  CAN message frame using 11-bit IDs.

EFF
Extended Frame Format
  CAN message frame using 29-bit IDs.

Address
CAN Address
CAN ID
Frame ID
  The 11- or 29-bit ID number at the start of a CAN data or remote frame.

CAN Frame
  See [](#frames).

DLC
Data Length Code
  The section of a CAN data frame preceding the data which determines how many bytes of data are present.

Serialisation
  The process of converting a data type into a byte sequence.

Deserialisation
  Reversing {term}`serialisation`.

Endianness
Byte order
  Whether computers store the bytes of a multi-byte type like a {expr}`uint32_t` in increasing or decreasing significance.

Little Endian
Little-Endian
Network Order
  {term}`Byte order` in which bytes are stored in order of least to most significant. Often referred to as network order, as it is often the preferred format for data transfer over networks. For example, `0x12345678` would be stored in memory as {cpp-inline}`[0x78 0x56 0x34 0x12]`.
  Reverse of {term}`Big-Endian`.

Big Endian
Big-Endian
  {term}`Byte order` in which bytes are stored in order of most to least significant. For example, `0x12345678` would be stored in memory as {cpp-inline}`[0x12 0x34 0x56 0x78]`.
  Reverse of {term}`Little-Endian`.

```
