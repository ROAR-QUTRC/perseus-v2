{
  stdenv,
  fetchYarnDeps,
  yarnConfigHook,
  yarnBuildHook,
  nodejs,
  corepack_24,
  bash,
  ...
}:
stdenv.mkDerivation rec {
  pname = "rover-ui";
  version = "0.0.1";
  src = ./.;
  yarnOfflineCache = fetchYarnDeps {
    yarnLock = src + "/yarn.lock";
    hash = "sha256-SYazzT70BsD8VUT30OkdmWbXCrryZO50IuF5D3ztg44=";
  };
  nativeBuildInputs = [
    yarnConfigHook
    yarnBuildHook
    nodejs
  ];
  propagatedBuildInputs = [ corepack_24 ];

  # yarnInstallPhase currently conflicts with how our project is set up,
  # so we define it directly, copying the entire build folder to the output
  # folder so we can make a script to run yarn start from there (useful on nix run)
  installPhase = ''
    runHook preInstall
    mkdir -p $out/build
    cp -r . $out/build
    # nix run searches for $out/bin/${pname} and tries to run it
    # so we put a bash script there which runs the webui
    mkdir -p $out/bin
    echo "#!${bash}/bin/bash
    cd $out/build
    ${corepack_24}/bin/yarn start" > $out/bin/${pname}
    chmod +x $out/bin/${pname}
    runHook postInstall
  '';
}
