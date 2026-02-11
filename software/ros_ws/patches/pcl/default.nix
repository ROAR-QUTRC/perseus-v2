{
  stdenv,
  fetchFromGitHub,
  lib,
  pkg-config,
  cmake,
  qt5,
  config,
  cudaPackages,
  eigen,
  libxt,
  libpcap,
  libusb1,
  nanoflann,
  llvmPackages,
  boost,
  flann,
  libpng,
  libtiff,
  qhull,
  vtk,
  ...
}:
stdenv.mkDerivation (finalAttrs: {
  pname = "pcl";
  version = "1.15.1";

  src = fetchFromGitHub {
    owner = "PointCloudLibrary";
    repo = "pcl";
    tag = "pcl-${finalAttrs.version}";
    hash = "sha256-+KyaajJM0I5CAcr8AiOLC4TkGV3Gm73a0/X8LQWFZMI=";
  };

  strictDeps = true;

  # remove attempt to prevent (x86/x87-specific) extended precision use
  # when SSE not detected
  postPatch = lib.optionalString (!stdenv.hostPlatform.isx86) ''
    sed -i '/-ffloat-store/d' cmake/pcl_find_sse.cmake
  '';

  nativeBuildInputs = [
    pkg-config
    cmake
    qt5.wrapQtAppsHook
  ]
  ++ lib.optionals config.cudaSupport [ cudaPackages.cuda_nvcc ];

  buildInputs = [
    eigen
    libxt
    libpcap
    qt5.qtbase
    libusb1
    nanoflann
  ]
  ++ lib.optionals stdenv.cc.isClang [ llvmPackages.openmp ];

  propagatedBuildInputs = [
    boost
    flann
    libpng
    libtiff
    qhull
    vtk
  ];

  cmakeFlags = [
    (lib.cmakeBool "BUILD_CUDA" config.cudaSupport)
    (lib.cmakeBool "BUILD_GPU" config.cudaSupport)
    (lib.cmakeBool "PCL_ENABLE_MARCHNATIVE" false)
    (lib.cmakeBool "WITH_CUDA" config.cudaSupport)
  ];
})
