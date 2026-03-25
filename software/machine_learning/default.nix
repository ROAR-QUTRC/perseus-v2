{
  python3Packages,
  ...
}:
python3Packages.buildPythonPackage {
  pname = "roar_ml";
  version = "0.0.1";
  src = ./.;
  pyproject = true;
  build-system = [  python3Packages.setuptools];

  propagatedBuildInputs = [
    python3Packages.torch
    python3Packages.pyyaml
    python3Packages.tqdm
  ];
}
