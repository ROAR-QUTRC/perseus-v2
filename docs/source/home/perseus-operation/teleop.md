# Perseus Teleoperation

This guide covers teleoperation of Perseus using a keyboard or Xbox controller. Teleoperation enables remote control of Perseus's movement. Operation of payloads (bucket, arms) is not covered in this guide.

## Prerequisites

The following conditions must be met before beginning:

- The laptop runs a Linux distribution with the latest code from the perseus-v2 Github repository:
  ```console
  git checkout main
  git pull
  ```
- Required fonts are installed for rendering the Orin's zsh shell with the [`powerlevel10k`](https://github.com/romkatv/powerlevel10k) theme (available in /perseus-v2/fonts). You will likely have to set the font to be your terminal's default (it'll come up as `MesloLGS NF`), though you may need to restart your terminal for the font to be available after installation.
- Xbox controller (if used) is paired with the laptop
- Perseus has sufficient battery charge

## Summary

1. Power on Perseus
2. Establish network availability
3. Connect laptop to network
4. Verify network connection
5. Access Perseus system
6. Manually confirm date/time
7. Launch Perseus software
8. Launch control software
9. Operate Perseus safely

### Perseus Power-up

1. Verify the E-stop is in the "up" (de-latched) position. Rotation may be necessary to unlatch if previously pressed.
2. Connect the battery's XT90 connector to the power meter.

### Network Setup

The Unifi UX is currently housed inside Perseus. Confirm that the power and network cables are connected to the Unifi UX. Once Perseus is powered on, the Unifi UX boot process should start automatically and take approximately 3 minutes. A complete boot is indicated when the screen displays a connected device counter instead of a progress bar and wifi devices can see the SSID "QUTRC-ROAR".

:::{warning}
The Unifi UX requires specific power specifications via its USB-C port. Use the onboard DC-DC power adaptor and avoid using externally grounded power sources.
:::

To enable Internet access, connect the Unifi UX's WAN port (marked with a globe symbol) to an Internet-enabled network via ethernet.

### Network Connection

Connect the laptop to the "QUTRC-ROAR" WiFi network using the provided credentials (ask in Discord).

### Network Verification

Perseus should automatically connect to the QUTRC-ROAR network. Verify the connection using one of these methods:

- Check the Unifi console for connected devices
- Use arp-scan on the laptop to list devices with allocated IP addresses:
  ```console
  sudo arp-scan --localnet
  ```

#### Network Troubleshooting

1. Physical Connectivity

   - Confirm Unifi UX is powered and fully booted (screen shows device count)
   - Verify power LED indicators are lit
   - Check E-Stop position as it affects system power

2. Laptop Connectivity

   - Verify connection to "QUTRC-ROAR" network
   - Check signal strength (maintain line of sight with Unifi UX if possible)
   - Confirm IP address allocation using `ip addr`

3. Perseus Connectivity

   ```console
   # Verify Perseus visibility
   ping big-brain.local

   # Alternative: network device scan
   sudo arp-scan --localnet | grep -i nvidia
   ```

### System Access

Access Perseus via SSH from the laptop:

```console
ssh qutrc@big-brain.local
```

:::{info}
You must use [`zellij`](https://zellij.dev/) on rover compute units when running multiple commands to prevent the need for multiple ssh connections.
:::

### Manually confirm the date/time

The Perseus compute modules (Orin, Pi5 etc) require appropriate date/time information. As they do not have a realtime backup clock source it is prudent to manually set the correct date/time after start up.

If the Unifi UX has access to the Internet then it is possible to sync the time with

```console
sudo chronyd -q
```

### Perseus Software Launch

Execute the following command on Perseus:

```console
nix run .#perseus
```

:::{warning}
Requires Internet access to complete this command, unless all necessary files have been previously downloaded to this specific device.
:::

### Control Software Launch

Launch the appropriate control software on the laptop based on the desired control method.

#### Keyboard Control

```console
cd perseus-v2
nix run .#ros2 -- run teleop_twist_keyboard teleop_twist_keyboard
```

Note: Control messages are only sent when the terminal has focus and receives keystrokes ({keys}`k` to stop).

#### Xbox Controller

:::{note}
Make sure that the Xbox controller connects to your laptop!
Sometimes you may need to re-pair, even if you've paired to it before.
:::

```console
cd perseus-v2
nix run .#xbox_controller
```

### Safe Operation

#### Safety Protocol

Before operating Perseus:

1. Ensure all bystanders:
   - Are aware of Perseus's operation
   - Know the E-Stop button location
   - Understand they should press the E-Stop if they observe:
     - Collision risks
     - Unsafe behaviour
     - Control issues
     - Any hazardous situations

The E-Stop, a red mushroom-shaped button on Perseus's top, immediately cuts all power when pressed.

:::{warning}
E-Stop activation causes immediate stopping. While this may cause abrupt deceleration, safety concerns always warrant E-Stop activation.
:::

#### Initial Testing

For Xbox controller input, the triggers act as dead man's switches.
The right trigger is for low-speed movement, and the left trigger is for high-speed movement (double the rate of low-speed by default).

Test Perseus's movement systematically:

1. Begin with slow speeds (right trigger only)
2. Test forward and backward motion separately from turning
3. Verify movement matches expected directions
