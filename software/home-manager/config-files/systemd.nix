{ ... }:
{
  xdg.configFile."systemd/network/80-can.network" = {
    # note: PresumeAck=yes prevents transmit errors with no other devices on the bus
    text = ''
      [Match]
      Name=can*

      [CAN]
      BitRate=500K
      RestartSec=1000ms
      BusErrorReporting=yes
      PresumeAck=yes
    '';
    # note: text block highlighted in orange
    onChange = ''
      cp -f ~/.config/systemd/network/80-can.network /etc/systemd/network/80-can.network 2&>1 /dev/null
      printf '\033[0;33m'
      echo "------------------------------"
      echo "Networking rules updated. Run \"sudo systemctl restart systemd-networkd\" for the changes to apply."
      echo "------------------------------"
      printf '\033[0m' # clear text formatting
    '';
  };
}
