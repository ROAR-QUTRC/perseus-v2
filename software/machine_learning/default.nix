{
  python3Packages,
  ...
}:
python3Packages.buildPythonPackage {
  pname = "roar_ml";
  version = "0.0.1";
  src = ./.;

  propagatedBuildInputs = [
    python3Packages.torch
  ];
}
