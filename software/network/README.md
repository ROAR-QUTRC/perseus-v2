# Network Configuration

The `ROAR-Network-Config.unifi` file contains the Unifi controller backup for the ROAR network.

To restore, import it via the Unifi controller at `https://unifi/network/default/dashboard`.

## Static IP Assignments

| Device             | IP Address       | Notes                       |
| ------------------ | ---------------- | --------------------------- |
| US 8 PoE Switch    | `192.168.1.TODO` | Base station switch         |
| U7 Long Range      | `192.168.1.TODO` | On-board Perseus AP         |
| U7 Outdoor Antenna | `192.168.1.TODO` | Base station directional AP |
| Livox LiDAR        | `192.168.1.TODO` | On-board Perseus            |

> **TODO:** Replace the IP addresses above with the actual static IPs from the Unifi config.

## DHCP Range

DHCP assigns addresses in the range `192.168.1.50` – `192.168.1.254` to avoid collisions with the static assignments above.

## Key Configuration

- The U7 Long Range is configured to only mesh-connect to the U7 Outdoor Antenna.
- Network SSID: `QUTRC-ROAR-LOCAL`
