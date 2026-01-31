(daisy-chain-pcb)=

# CAN Bus Daisy Chain PCB

This PCB is designed to connect various different CAN bus conductor pairs to make wiring the bus easier.
For terminology and the rationale behind _why_ this is needed, see <project:#can-loops-branches>.

Since it was originally designed for CAN wiring to ESCs, the PCB also includes power distribution in the form of a male XT90 connector to 3x female XT30 connectors.

## Configurations

To allow for different uses, the PCB has several jumper pairs which, when bridged, put it into different configurations.
The design configurations are listed below.

| Configuration          | Jumpers bridged    | Main A    | Main B    | Multi 1 B |
| ---------------------- | ------------------ | --------- | --------- | --------- |
| Loop Splitter          | J9, J10            | Multi 1 A | Multi 2 A | N/C       |
| Splitter with loopback | J9, J10, J13, J14  | Multi 1 A | Multi 2 A | Multi 2 A |
| 2x Looping Branch      | J11, J12, J13, J14 | Multi 1 A | Multi 2 B | Multi 2 A |
| 3x Looping Branch      | J13, J14           | Multi 1 A | Multi 3 B | Multi 2 A |
| Main Loopback          | J7, J8             | Main B    | Main A    | N/C       |
| Multi 1 Loopback       | J5, J6             | Multi 1 A | Multi 3 B | Multi 1 A |

For completeness, the jumper pairs are as follows:

| Jumper 1 | Jumper 2 | Connection 1 | Connection 2 |
| -------- | -------- | ------------ | ------------ |
| J5       | J6       | Multi 1 A    | Multi 1 B    |
| J7       | J8       | Main A       | Main B       |
| J9       | J10      | Multi 2 A    | Main B       |
| J11      | J12      | Multi 2 B    | Main B       |
| J13      | J14      | Multi 1 B    | Multi 2 A    |

And the fixed connections:

| Pair 1    | Pair 2    |
| --------- | --------- |
| Main A    | Multi 1 A |
| Main B    | Multi 3 B |
| Multi 2 B | Multi 3 A |

### Loop Splitter

This is the configuration for all daisy-chain boards _inside_ the rover, and usually the most useful.
It splits a single {term}`loop` connection (both "A" and "B") into two standard ("A" pair only) connections.

```{warning}
If the optional J13/J14 pair are connected, the J5/J6 pin jumpers can be used to create a convenient loopback across the Main connector.
However, bridging those jumpers _may_ reduce performance by adding {term}`branch` length.
Additionally, if J5/J6 are in use, it would be easy to forget what state they're in, which could cause unexpected behaviour and hard to debug issues.
```

#### Connectors

- Main: Loop
- Multi 1: Standard
- Multi 2: Standard
- Multi 3: N/A

### 2x Looping Branch

This was used to connect Perseus V1's ESCs on the suspension arm to the CAN bus.
Main, Multi 1, and Multi 2 are all loops in this configuration.

#### Connectors

- Main: Loop
- Multi 1: Loop
- Multi 2: Loop
- Multi 3: N/A

### 3x Looping Branch

This configuration has never been used, and was intended to allow for fallback to the Artemis chassis with its 3 wheels (and therefore ESCs) per side.
As such, all 4 connectors are loops.

#### Connectors

- Main: Loop
- Multi 1: Loop
- Multi 2: Loop
- Multi 3: Loop

### Loopback

Intended only to complete a branch's loop.
Currently this job is handled by loopback connectors instead and is not needed.

#### Connectors

- Main: Loopback
- Multi 1: N/A
- Multi 2: N/A
- Multi 3: N/A

## Resources

- Schematic: [CAN Bus Daisy Chain PCB](/_static/CAN_Bus_Daisy_Chain.pdf)
