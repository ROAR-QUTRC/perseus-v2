# Powering on Perseus

This guide explains the correct process to power on persues.

## Prerequisites

- Charged batteries
- Check cable connections (Including the can jump on the front of Perseus)
- Know the difference between drive stop button (green rectangle) and power button (round silver)

## Guide

### Pre-Power on Checks

1. Verify that the E-stop is "up". This means it has been rotated clockwise until it is delatched.
2. Verify that the drive stop is "up". This means that it is level with the top of the panel it is mounted in.
3. Connect the battery's XT90 connecter to the power meter inside of Perseus.

### Power on Rover Control Board

The Rover Control Board (RCB) is the large purple PCB. To power the RCB press and hold the power button until the contactor is enabled. This should take about one second and will make an audible click when the contactor is engaged. Once the power button is flashing blue, the rover control board is powered.

:::{note}
The power button has three status modes:

| State    | Meaning                              |
| -------- | ------------------------------------ |
| Off      | Contactor is disabled                |
| Flashing | Contactor is enabled without compute |
| Solid    | Compute is enabled                   |

:::

1. Press and hold the power button until the contactor is enabled. This should take about one second and will make an audible click when the contactor has been engaged.
2. Once the power button is flashing blue the rover control board is successfully powered on.

### Power Rover Systems

:::{info}

Each bus has an orange and red LED next to its terminal to display status.
These are not controlled with software and instead directly display hardware status. The red button indicates the precharge status, and the orange button indicates whether the bus is connected.

If the orange LED is on, then the bus is powered. The red LED should periodically flash when attempting to precharge. In a worst-case situation, the red LED may be solidly lit, signifying a major fault, and the rover must be e-stopped IMMEDIATELY.

:::

#### Compute Power

Once Perseus is powered the first bus that should be powered is compute as this is required for powering on subsequent busses. This is done by pressing the power button once and waiting for the compute bus LED on the RCB to light. Before compute can be powered off the orin MUST be powered off with the command `sudo poweroff`, then to power off this bus you must hold down the power button.

#### Bus Power

To enable an individual bus on the RCB once compute is powered, the power button must be pressed a specific number of times in quick succession. Refer to the table below for the number of presses per bus:

| Bus   | Number of presses |
| ----- | ----------------- |
| Drive | 1                 |
| Aux   | 2                 |
| Spare | 3                 |

Note: Drive needs the drive stop button to be "down" additionally to bus power.

## Light Tower Status

### Bands

Each band represents a different bus on the rover. Going from the top down:

- Contactor (Largest band)
- Compute
- Drive
- Aux
- Spare

### Colours

Each colour represents a specific status:

- Black - Off
- Green - On
- Yellow - Precharging
- Red - Short-circuit
- Flashing tangerine - Switch failed
- Blue - Overload
- Flashing red - Fault
