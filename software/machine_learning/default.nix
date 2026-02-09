{ 
  setuptools,
  python3Packages,
  ...
}:
python3Packages.buildPythonPackage {
  pname = "roar-ml";
  version = "0.0.1";
  src = ./.;
  
  build-system = [
    setuptools
  ];

  propagatedBuildInputs = [
    python3Packages.torch
  ];
}




