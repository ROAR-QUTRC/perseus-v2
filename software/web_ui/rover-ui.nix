{
  stdenv,
  fetchYarnDeps,
  yarnConfigHook,
  yarnBuildHook,
  nodejs_24,
  corepack_24,
  bash,
  gst_all_1,
  libnice,
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
    nodejs_24
  ];
  propagatedBuildInputs = [
    corepack_24
    libnice
  ]
  ++ (with gst_all_1; [
    gstreamer
    gst-plugins-bad
    gst-plugins-base
    gst-plugins-good
    gst-plugins-rs
  ]);

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
