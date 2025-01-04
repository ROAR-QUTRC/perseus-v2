{
  home-manager,
  nixpkgs,
  nixpkgs-unstable,
  ...
}@inputs:
let
  # build a configuration:
  # - Loads the current home.nix file which in turn loads all the default config
  # - If a hostname is provided, loads the machine-specific config from machines/${hostname}/default.nix
  # - Adds the unstable nixpkgs overlay so unstable packages can be accessed from `pkgs.unstable`
  buildConfig =
    {
      system,
      hostname ? "",
      username,
      ...
    }@homeConfig:
    let
      pkgs = nixpkgs.legacyPackages.${system};
      pkgs-unstable = nixpkgs-unstable.legacyPackages.${system};
      configAttrName = if hostname != "" then "${username}@${hostname}" else username;
      machineConfig = ./machines/${hostname}/default.nix;
    in
    {
      ${configAttrName} = home-manager.lib.homeManagerConfiguration {
        inherit pkgs;

        modules =
          [
            (
              { ... }:
              {
                nixpkgs.overlays = [ (final: prev: { unstable = pkgs-unstable; }) ];
              }
            )
            ./home.nix
          ]
          ++ pkgs.lib.lists.optional ((hostname != "") && (builtins.pathExists machineConfig)) machineConfig;
        extraSpecialArgs = {
          inherit
            inputs
            hostname
            username
            homeConfig
            ;
        };
      };
    };
  # Build a set of machine configurations from a list of settings
  buildConfigs =
    # foldl' is just like array reduce in other languages - args are accumulator function,
    # initial value, and list to reduce
    configs: builtins.foldl' (acc: new: acc // new) { } (builtins.map buildConfig configs);
in
buildConfigs [
  # most systems for the rover (with the user qutrc) are ARM64,
  # so better to default to it, and specify x86 systems manually
  {
    system = "aarch64-linux";
    username = "qutrc";
  }
  # and systems with manual config should also be specified
  {
    system = "aarch64-linux";
    username = "qutrc";
    hostname = "big-brain";
  }
  {
    system = "aarch64-linux";
    username = "qutrc";
    hostname = "medium-brain";
  }
  {
    system = "x86_64-linux";
    username = "qutrc";
    hostname = "gcs";
  }
]
