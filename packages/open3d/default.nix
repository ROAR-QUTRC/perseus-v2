{
  lib,
  stdenv,
  fetchFromGitHub,
  fetchurl,
  cmake,
  ninja,
  pkg-config,
  python3,
  python3Packages,
  eigen,
  fmt,
  embree,
  glew,
  glfw,
  libjpeg,
  libpng,
  jsoncpp,
  nanoflann,
  qhull,
  openssl,
  curl,
  zeromq,
  cppzmq,
  libsodium,
  vtk,
  minizip,
  xorg,
  librealsense,
  libGL,
  unzip,
}:

let
  # Pre-fetch third-party dependencies not available or incompatible in nixpkgs.
  # URLs and hashes taken directly from Open3D's cmake files.

  tbb-src = fetchurl {
    url = "https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.12.0.tar.gz";
    hash = "sha256-x7t6ppwlTZG48AQaccW8w5Nqy2RAihcZrsCyt2Od2E8=";
  };

  assimp-src = fetchurl {
    url = "https://github.com/assimp/assimp/archive/refs/tags/v5.4.2.zip";
    hash = "sha256-A+ONEj9r8ZpIZY0Zf9CcmmnbiMB2tWpHarLan164fcw=";
  };

  tinyobjloader-src = fetchurl {
    url = "https://github.com/tinyobjloader/tinyobjloader/archive/refs/tags/v2.0.0rc8.tar.gz";
    hash = "sha256-uMly37vO8z1VVU58kDGr5wQHlbZ3eK02YKUK+n327FY=";
  };

  tinygltf-src = fetchurl {
    url = "https://github.com/syoyo/tinygltf/archive/72f4a55edd54742bca1a71ade8ac70afca1d3f07.tar.gz";
    hash = "sha256-noSNzw7H3LNSzteCrqMgZKY6UbPGjtFMaFMeCGMqLZA=";
  };

  poissonrecon-src = fetchurl {
    url = "https://github.com/isl-org/Open3D-PoissonRecon/archive/90f3f064e275b275cff445881ecee5a7c495c9e0.tar.gz";
    hash = "sha256-ExDfDID/Bha4/Pmy+1aKqbIZDQ4HGw6tR9ujOcFGsdM=";
  };

  msgpack-src = fetchurl {
    url = "https://github.com/msgpack/msgpack-c/releases/download/cpp-3.3.0/msgpack-3.3.0.tar.gz";
    hash = "sha256-bhFNEqXduMsR9mn4PzIkbkhKit3QzpPydJlvGUHB8Hs=";
  };

  uvatlas-src = fetchurl {
    url = "https://github.com/microsoft/UVAtlas/archive/refs/tags/may2022.tar.gz";
    hash = "sha256-WRUWkToPPDgfH9AWR8sbjR7q3lddHHJq6PXdn4O4F1Q=";
  };

  directxheaders-src = fetchurl {
    url = "https://github.com/microsoft/DirectX-Headers/archive/refs/tags/v1.606.3.tar.gz";
    hash = "sha256-vwGDmB5QUzbpGGCTdJB8k0uZ62HAgm11pWSfQVaKvEs=";
  };

  directxmath-src = fetchurl {
    url = "https://github.com/microsoft/DirectXMath/archive/refs/tags/may2022.tar.gz";
    hash = "sha256-ssW0GcosVnhg98IEycCJBXPopYyNh3Rz5PO6a4UcpM4=";
  };

  mkl-src = fetchurl {
    url = "https://github.com/isl-org/open3d_downloads/releases/download/mkl-static-2024.1/mkl_static-2024.1.0-linux_x86_64.tar.xz";
    hash = "sha256-83yUQOPWZNIYiaRgfv/NR0crzONH2mwr/HqumRlxtJk=";
  };

  ipp-src = fetchurl {
    url = "https://github.com/isl-org/open3d_downloads/releases/download/mkl-static-2024.1/ipp_static-2021.11.0-linux_x86_64.tar.xz";
    hash = "sha256-UfM/1b9QEenq4OA05cxwp8CsC6k9aj9m/X4UXPGl4ws=";
  };

  inherit (python3) pythonVersion; # e.g. "3.13"

in
stdenv.mkDerivation rec {
  pname = "open3d";
  version = "0.19.0";

  src = fetchFromGitHub {
    owner = "isl-org";
    repo = "Open3D";
    rev = "v${version}";
    hash = "sha256-jWjtfDcjDBOQHH4s2e1P8ye19JlucYIZPi0pgvOsdcA=";
  };

  nativeBuildInputs = [
    cmake
    ninja
    pkg-config
    python3
    unzip
  ];

  buildInputs = [
    eigen
    fmt
    embree
    glew
    glfw
    libjpeg
    libpng
    jsoncpp
    nanoflann
    qhull
    openssl
    curl
    zeromq
    cppzmq
    libsodium
    vtk
    minizip
    python3Packages.pybind11
    xorg.libX11
    xorg.libXrandr
    xorg.libXinerama
    xorg.libXcursor
    xorg.libXi
    librealsense
    libGL
  ];

  propagatedBuildInputs = [
    python3Packages.numpy
  ];

  passthru.pythonDeps =
    ps: with ps; [
      numpy
      plotly
      dash
    ];

  # Patch download URLs to use pre-fetched local files from the Nix store
  preConfigure = ''
    substituteInPlace 3rdparty/mkl/tbb.cmake \
      --replace-fail \
      'https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.12.0.tar.gz' \
      'file://${tbb-src}'

    substituteInPlace 3rdparty/assimp/assimp.cmake \
      --replace-fail \
      'https://github.com/assimp/assimp/archive/refs/tags/v5.4.2.zip' \
      'file://${assimp-src}'

    substituteInPlace 3rdparty/tinyobjloader/tinyobjloader.cmake \
      --replace-fail \
      'https://github.com/tinyobjloader/tinyobjloader/archive/refs/tags/v2.0.0rc8.tar.gz' \
      'file://${tinyobjloader-src}'

    substituteInPlace 3rdparty/tinygltf/tinygltf.cmake \
      --replace-fail \
      'https://github.com/syoyo/tinygltf/archive/72f4a55edd54742bca1a71ade8ac70afca1d3f07.tar.gz' \
      'file://${tinygltf-src}'

    substituteInPlace 3rdparty/possionrecon/possionrecon.cmake \
      --replace-fail \
      'https://github.com/isl-org/Open3D-PoissonRecon/archive/90f3f064e275b275cff445881ecee5a7c495c9e0.tar.gz' \
      'file://${poissonrecon-src}'

    substituteInPlace 3rdparty/msgpack/msgpack_build.cmake \
      --replace-fail \
      'https://github.com/msgpack/msgpack-c/releases/download/cpp-3.3.0/msgpack-3.3.0.tar.gz' \
      'file://${msgpack-src}'

    substituteInPlace 3rdparty/mkl/mkl.cmake \
      --replace-fail \
      'https://github.com/isl-org/open3d_downloads/releases/download/mkl-static-2024.1/mkl_static-2024.1.0-linux_x86_64.tar.xz' \
      'file://${mkl-src}'

    substituteInPlace 3rdparty/ipp/ipp.cmake \
      --replace-fail \
      'https://github.com/isl-org/open3d_downloads/releases/download/mkl-static-2024.1/ipp_static-2021.11.0-linux_x86_64.tar.xz' \
      'file://${ipp-src}'

    # UVAtlas: replace URL and git clones with pre-fetched local tarballs
    substituteInPlace 3rdparty/uvatlas/uvatlas.cmake \
      --replace-fail \
        'https://github.com/microsoft/UVAtlas/archive/refs/tags/may2022.tar.gz' \
        'file://${uvatlas-src}' \
      --replace-fail \
        'GIT_REPOSITORY https://github.com/microsoft/DirectX-Headers.git' \
        'URL file://${directxheaders-src}' \
      --replace-fail \
        'GIT_TAG v1.606.3' \
        '# patched: tag removed' \
      --replace-fail \
        'GIT_REPOSITORY https://github.com/microsoft/DirectXMath.git' \
        'URL file://${directxmath-src}' \
      --replace-fail \
        'GIT_TAG may2022' \
        '# patched: tag removed'
    # Remove GIT_SHALLOW (appears twice, once per git dep)
    sed -i '/GIT_SHALLOW TRUE/d' 3rdparty/uvatlas/uvatlas.cmake
  '';

  cmakeFlags = [
    # Build configuration
    "-DBUILD_PYTHON_MODULE=ON"
    "-DBUILD_GUI=OFF"
    "-DBUILD_CUDA_MODULE=OFF"
    "-DBUILD_ISPC_MODULE=OFF"
    "-DBUILD_WEBRTC=OFF"
    "-DBUILD_EXAMPLES=OFF"
    "-DBUILD_UNIT_TESTS=OFF"
    "-DBUILD_BENCHMARKS=OFF"

    # LibRealSense support
    "-DBUILD_LIBREALSENSE=ON"
    "-DUSE_SYSTEM_LIBREALSENSE=ON"

    # Use system dependencies where compatible
    "-DUSE_SYSTEM_EIGEN3=ON"
    "-DUSE_SYSTEM_FMT=ON"
    "-DUSE_SYSTEM_PYBIND11=ON"
    "-DUSE_SYSTEM_NANOFLANN=ON"
    "-DUSE_SYSTEM_GLEW=ON"
    "-DUSE_SYSTEM_GLFW=ON"
    "-DUSE_SYSTEM_JPEG=ON"
    "-DUSE_SYSTEM_PNG=ON"
    "-DUSE_SYSTEM_JSONCPP=ON"
    "-DUSE_SYSTEM_QHULLCPP=ON"
    "-DUSE_SYSTEM_EMBREE=ON"
    "-DUSE_SYSTEM_ZEROMQ=ON"
    "-DUSE_SYSTEM_OPENSSL=ON"
    "-DUSE_SYSTEM_CURL=ON"
    "-DUSE_SYSTEM_VTK=ON"

    # Python configuration
    "-DPYTHON_EXECUTABLE=${python3}/bin/python3"
    "-DPython3_EXECUTABLE=${python3}/bin/python3"
    "-DPython3_INCLUDE_DIR=${python3}/include/python${pythonVersion}"
    "-DPython3_LIBRARY=${python3}/lib/libpython${pythonVersion}.so"

    # Install configuration - use relative libdir to avoid breaking ExternalProject paths
    "-DCMAKE_INSTALL_PREFIX=${placeholder "out"}"
    "-DCMAKE_INSTALL_LIBDIR=lib"
  ];

  # Open3D tries to detect git version; provide a fallback since we build from a tarball
  OPEN3D_VERSION = version;

  # minizip headers are in a subdirectory but Open3D expects them at top level
  NIX_CFLAGS_COMPILE = "-isystem ${minizip}/include/minizip";
  # OpenSSL's libcrypto is needed by curl but not on the link line
  NIX_LDFLAGS = "-lcrypto";

  # Build the Python package target (not part of the default build)
  postBuild = ''
    ninja python-package
  '';

  postInstall = ''
    # Install the assembled Python package to site-packages
    local pydir="$out/lib/python${pythonVersion}/site-packages"
    mkdir -p "$pydir"
    cp -r lib/python_package/open3d "$pydir/open3d"
  '';

  meta = with lib; {
    description = "Open3D: A Modern Library for 3D Data Processing";
    homepage = "https://www.open3d.org/";
    license = licenses.mit;
    platforms = [ "x86_64-linux" ];
  };
}
