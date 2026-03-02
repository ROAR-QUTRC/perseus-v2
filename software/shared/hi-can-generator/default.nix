{ python3Packages, ... }:
python3Packages.buildPythonApplication {
  pname = "hi-can-generator";
  version = "0.0.1";
  src = ./.;
  pyproject = true;
  build-system = [ python3Packages.setuptools ];
}
