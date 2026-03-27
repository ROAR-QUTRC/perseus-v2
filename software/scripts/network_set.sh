#!/usr/bin/env bash

IP_A=$(ip a)
INTERFACE_LINES=$(echo "$IP_A" | grep -e "[0-9]: ")
INTERFACES_COLON=$(echo "$INTERFACE_LINES" | awk '{print $2}')
INTERFACES=$(echo "$INTERFACES_COLON" | awk 'BEGIN {FS = ":"} ; {print $1}')
INTERFACES_NO_LO=$(echo "$INTERFACES" | grep -v "lo")

cd "$(git rev-parse --show-toplevel)" || exit
mkdir -p config
echo "<CycloneDDS>
  <Domain>
    <General>
      <!-- Avoid broadcasting data to all client  -->
      <AllowMulticast>false</AllowMulticast> " >config/cyclonedds_autonomy.xml
for INTERFACE in $INTERFACES_NO_LO; do
        echo "      <NetworkInterfaceAddress>$INTERFACE</NetworkInterfaceAddress>" >>config/cyclonedds_autonomy.xml
done
echo "    </General>

    <Discovery>
      <Peers>
        <!-- Robot PC -->
        <!-- qutrc@big-brain IP -->
        <Peer address=\"192.168.2.11\"/> 
        <!-- qutrc@medium-brain IP -->
        <Peer address=\"192.168.2.12\"/>
        <!-- qutrc@small-brain -->
        <Peer address=\"192.168.2.15\"/>


         <!-- Base Station -->
         <!-- Kelvin's Machine -->
        <!-- <Peer address=\"192.168.2.13\"/>  -->
         <!-- Luca's Machine -->
        <!-- <Peer address=\"192.168.2.14\"/> -->
         <!-- Dan's Machine -->
        <Peer address=\"192.168.2.50\"/>
        <!-- Kelvin's eth0 -->
        <Peer address=\"192.168.2.21\"/>
        <!-- Brock's -->
        <Peer address=\"192.168.2.181\"/>
        <!-- Shaans -->
        <!-- <Peer address=\"192.168.2.140\"/> -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>" >>config/cyclonedds_autonomy.xml
