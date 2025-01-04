{ pkgs, ... }:
{
  home.packages = with pkgs; [
    wishlist
    tldr
    iperf3
  ];

  programs.btop = {
    enable = true;
    settings = {
      update_ms = 200;
      cpu_graph_lower = "user";
    };
  };
}
