{ python3Packages, ... }:
python3Packages.buildPythonApplication {
  pname = "hi-can-generator";
  pyproject = false;
  version = "0.0.1";
  src = ./.;
  build-system = [ python3Packages.setuptools ];
}
