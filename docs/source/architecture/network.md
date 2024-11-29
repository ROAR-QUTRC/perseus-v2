# Network

## TLDR

Perseus assumes that it will connect to a wifi network with SSID: "QUTRC-ROAR".

All processes and nodes running should be able to operate via a network connection sharing a single 2.4GHz wifi connection with a 20MHz channel width.

Whilst the rules allow for 40Mhz bonding interference is possible.

Assuming 20MHz means 802.11g which is 54Mbps.

This equates to a maximum usage network connection of 6.75 MBps.

If bonding is used then 802.11n allows for up to 600 Mbps (this would require 4xMIMO) being 75MBps.

## Purpose

Some of the software written for Perseus will operate better if details of the network Perseus operates on are known.
This file seeks to capture these details

## Wifi Network Assumptions

The wifi network that Perseus can see has the following attributes
SSID for 2.4Ghz: "QUTRC-ROAR"
SSID for 5Ghz: "QUTRC-ROAR"
Hard-wired ethernet operation also needs to be accommodated

During competition:
Either 20 Mhz band on nominated Channel 1 or Channel 11.
OR
40Mhz band on either 1-5 or 9-13

During Practice:
20 or 40 Mhz band centred on channel 6

Assumption: All systems need to be able to concurrently operate on 20Mhz (Max ~97 Mbps)
