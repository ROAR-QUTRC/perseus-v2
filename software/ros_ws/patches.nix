rosDistro: final: prev:
let
  rosOverlay =
    rosFinal: rosPrev:
    let
      patchLidarSlamLicense =
        package:
        (package.overrideAttrs (
          {
            meta ? { },
            ...
          }:
          {
            meta = meta // {
              license = [ prev.lib.licenses.bsd2 ];
            };
          }
        ));
    in
    {
      fields2cover =
        let
          nlohmann_json = final.nlohmann_json.overrideAttrs {
            src = final.fetchFromGitHub {
              owner = "nlohmann";
              repo = "json";
              rev = "v3.10.5";
              sha256 = "sha256-DTsZrdB9GcaNkx7ZKxcgCA3A9ShM5icSF0xyGguJNbk=";
            };
            doCheck = false;
          };
          tinyxml-2 = final.tinyxml-2.overrideAttrs {
            src = final.fetchFromGitHub {
              owner = "leethomason";
              repo = "tinyxml2";
              rev = "c2d30872e20621955ca7feb9168bad996d591a19";
              sha256 = "sha256-Gn4d6v7p60XRam2wclaSFAiAxmNgKAKPxRCEmcMtJIE=";
            };
          };
          steering-functions = final.stdenv.mkDerivation {
            name = "steering-functions";
            src = final.fetchFromGitHub {
              owner = "Fields2Cover";
              repo = "steering_functions";
              rev = "33fc010017efa1ef2c8a2d4779fcda94b4b30d20";
              sha256 = "sha256-IaqvFWQFgU6yGOrvCrz7c7TEH4+vFf34iZ5qBZ1yMUw=";
            };
            nativeBuildInputs = with final; [
              cmake
            ];
            propagatedBuildInputs = with final; [
              eigen
            ];

            patches = [
              ./patches/fields2cover/steering-functions.patch
            ];
          };
          spline = final.stdenv.mkDerivation {
            name = "spline";
            # no version!

            src = final.fetchFromGitHub {
              owner = "Fields2Cover";
              repo = "spline";
              rev = "1b5d4bad29082997076b264de84ca6d46c2ae6ab";
              sha256 = "sha256-nWaanoqOVzDlxbfWFjT4j6ZQtp76E9tVbNVvx1R2dlM=";
            };

            nativeBuildInputs = with final; [
              cmake
            ];

            patches = [
              ./patches/fields2cover/spline.patch
            ];

            meta = {
              description = "c++ cubic spline library";
              license = with final.lib.licenses; [ gpl2 ];
            };
          };
          matplotlib-cpp = final.stdenv.mkDerivation {
            name = "matplotlib-cpp";
            # also no version!
            src = final.fetchFromGitHub {
              owner = "Fields2Cover";
              repo = "matplotlib-cpp";
              rev = "75c15d0c907a4b68bca5ef97032302bd14ccab8e";
              sha256 = "sha256-6rUm40oJaA8V9z1lj3HIC/TdZAp9GGEUPC0OvdESOMI=";
            };

            nativeBuildInputs = with final; [
              cmake
            ];
            propagatedBuildInputs = with final; [
              python3
              python3Packages.matplotlib
            ];

            patches = [
              ./patches/fields2cover/matplotlib-cpp.patch
            ];
          };
        in
        rosPrev.fields2cover.overrideAttrs (
          {
            patches ? [ ],
            postPatch ? "",
            ...
          }:
          {
            version = "1.2.1";
            src = final.fetchFromGitHub {
              owner = "ros2-gbp";
              repo = "fields2cover-release";
              rev = "upstream/1.2.1";
              sha256 = "sha256-g8LitJGD3iceBB2j10jdcNQWc3qE3JoSKe6oYsDR/CU=";
            };
            propagatedBuildInputs =
              with rosFinal;
              with final;
              [
                boost
                eigen
                gdal
                geos
                git
                gtest
                python3
                python3Packages.tkinter
                swig
                tbb_2021_11
                nlohmann_json
                tinyxml-2
                steering-functions
                spline
                matplotlib-cpp
              ];
            buildInputs = with final; [ or-tools ];
            patches = patches ++ [ ./patches/fields2cover/fields2cover.patch ];
            postPatch = ''
              ${postPatch}
              find . -type f -exec sed -i -e "s,gdal/,,g" {} \;
            '';
          }
        );

      opennav-coverage = rosPrev.opennav-coverage.overrideAttrs (
        {
          propagatedBuildInputs ? [ ],
          ...
        }:
        {
          propagatedBuildInputs = propagatedBuildInputs ++ [ rosFinal.visualization-msgs ];
        }
      );

      livox-ros-driver2 = rosPrev.livox-ros-driver2.overrideAttrs (
        {
          buildInputs ? [ ],
          patches ? [ ],
          ...
        }:
        {
          buildInputs = buildInputs ++ [ final.livox-sdk2 ];
          patches = patches ++ [
            ./patches/livox-ros-driver2/rename-files.patch
            ./patches/livox-ros-driver2/livox-ros-driver2.patch
          ];
        }
      );

      # rviz-ogre-vendor builds OGRE in a nested CMake ExternalProject that does
      # not inherit the outer package's search paths, so on aarch64 OGRE's
      # find_package(ZLIB) resolves to the host /usr/lib/aarch64-linux-gnu/libz.so,
      # which Nix's purity check rejects ("impure path used in link"), breaking
      # the ARM build. OGRE's args set CMP0074=NEW, so setting the ZLIB_ROOT
      # environment variable (which the ExternalProject *does* inherit) forces
      # find_package(ZLIB) to the store copy. The symlinkJoin gives ZLIB_ROOT a
      # single prefix containing both lib/ (zlib) and include/ (zlib.dev).
      rviz-ogre-vendor = rosPrev.rviz-ogre-vendor.overrideAttrs (
        {
          buildInputs ? [ ],
          ...
        }:
        {
          buildInputs = buildInputs ++ [ final.zlib ];
          ZLIB_ROOT = final.symlinkJoin {
            name = "zlib-root";
            paths = [
              final.zlib
              final.zlib.dev
            ];
          };
        }
      );

      perseus-input = rosPrev.perseus-input.overrideAttrs (
        {
          propagatedBuildInputs ? [ ],
          ...
        }:
        {
          propagatedBuildInputs = final.lib.remove rosFinal.perseus-input-config propagatedBuildInputs;
        }
      );
      perseus-vision =
        let
          cubeDetectorModel = prev.fetchurl {
            url = "https://github.com/ROAR-QUTRC/perseus-v2/releases/download/models-v1/cube_detector_yolob8s.onnx";
            sha256 = "sha256-EkWhKFYog5ysSobcE4DFW2S8j3ZLQZDBxucWLa/KVfc=";
          };
        in
        rosPrev.perseus-vision.overrideAttrs (
          {
            postUnpack ? "",
            ...
          }:
          {
            postUnpack = postUnpack + ''
              mkdir -p $sourceRoot/models
              cp ${cubeDetectorModel} $sourceRoot/models/cube_detector_yolob8s.onnx
            '';
          }
        );
      # NOTE: fast-lio needs no override here. The C++17 bump and the configurable
      # TF frames used to be carried as patches in ./patches/fast_lio; they now live
      # in the ROAR fork itself (see ../third-party-packages/fast-lio/default.nix),
      # so the frames are set through the `common.*_frame` / `publish.tf_*` params
      # in the config YAML instead of a source patch.
      lidarslam-msgs = patchLidarSlamLicense (
        rosPrev.lidarslam-msgs.overrideAttrs (
          {
            buildInputs ? [ ],
            nativeBuildInputs ? [ ],
            ...
          }:
          {
            buildInputs = buildInputs ++ [ rosFinal.ament-cmake-auto ];
            nativeBuildInputs = nativeBuildInputs ++ [ rosFinal.ament-cmake-auto ];
          }
        )
      );
      ndt-omp-ros2 = rosPrev.ndt-omp-ros2.overrideAttrs (
        {
          patches ? [ ],
          buildInputs ? [ ],
          propagatedBuildInputs ? [ ],
          ...
        }:
        let
          runtimePackages = [
            rosFinal.pcl
            rosFinal.rclcpp
            rosFinal.std-msgs
          ];
        in
        {
          patches = patches ++ [ ./patches/ndt-omp-ros2/boost.patch ];
          buildInputs = prev.lib.subtractLists runtimePackages buildInputs;
          propagatedBuildInputs = propagatedBuildInputs ++ runtimePackages;
        }
      );
      graph-based-slam = patchLidarSlamLicense (
        rosPrev.graph-based-slam.overrideAttrs (
          {
            propagatedBuildInputs ? [ ],
            ...
          }:
          {
            propagatedBuildInputs = propagatedBuildInputs ++ [
              rosFinal.nav-msgs
              rosFinal.std-srvs
            ];
          }
        )
      );
      # mesh_map (mesh_navigation) unconditionally #includes
      # lvr2/algorithm/ClosestSurfacePoint.hpp, added upstream 2026-07-29 (commit 3dd439e) --
      # after the jazzy release (25.2.1/25.2.2) was cut, so the released lvr2 simply doesn't
      # have it. lvr2 carries no tags at all; mesh_navigation's own source_dependencies.yaml
      # already documents lvr2 as unreleasable-via-rosdep and builds it from `main` in CI, so
      # we pin `main`'s tip the same way rather than the rosdistro release. The header itself
      # is a small, dependency-free abstract interface (no Embree/CUDA needed to compile it --
      # Embree stays a `find_package(embree QUIET)` optional in lvr2's own CMakeLists.txt).
      #
      # To move the pin, push a newer commit and re-read both values from:
      #   nix-prefetch-git --url https://github.com/uos/lvr2 --rev <commit>
      #
      # lvr2's CMakeLists.txt calls find_package(PkgConfig REQUIRED) to locate FreeImage,
      # but the released package's ros2nix-generated recipe doesn't request pkg-config as a
      # native build input -- CMake configure fails outright without it.
      #
      # It also unconditionally find_package(OpenGL)s and links the core library against
      # ${OPENGL_LIBRARIES}, with no buildInput providing it. lvr2_mesh_reducer and
      # lvr2_hdf5_mesh_tool (under LVR2_BUILD_TOOLS, on by default) then hard-fail configure
      # by feeding the resulting "OPENGL_INCLUDE_DIR-NOTFOUND" placeholder to
      # target_include_directories(), which rejects non-absolute paths. We only need the
      # core library (mesh_msgs_conversions/mesh_layers link against it, not any CLI tool),
      # so disable LVR2_BUILD_TOOLS entirely and add libGL/libGLU so the core lib's own
      # find_package(OpenGL) and its display/*.cpp's GL/glu.h actually succeed, instead of
      # merely avoiding the crash.
      #
      # lvr2's exported lvr2-config.cmake also unconditionally find_dependency(MPI)s, even
      # though nothing in the actual build needs it (lvr2's own configure only ever logs
      # "Could NOT find MPI" as informational, and adding a real MPI package here just trips
      # a *different* branch of lvr2's own CMakeLists.txt that then demands a boost_mpi build
      # variant nixpkgs' boost isn't built with). It's vestigial, so consumers -- e.g.
      # mesh_msgs_conversions, via find_package(lvr2 REQUIRED) -- shouldn't inherit it as
      # REQUIRED either. Strip the line from the installed config instead of satisfying it.
      lvr2 = rosPrev.lvr2.overrideAttrs (
        {
          nativeBuildInputs ? [ ],
          propagatedBuildInputs ? [ ],
          cmakeFlags ? [ ],
          postPatch ? "",
          postInstall ? "",
          ...
        }:
        {
          version = "2026-08-06";
          src = final.fetchFromGitHub {
            owner = "uos";
            repo = "lvr2";
            rev = "05a197e2914006555e32481679d80b77f8f8637a";
            hash = "sha256-8CG4/VemKbtKKsOJxIPkvdxcUCew1fSO6ClqkVow+8I=";
          };
          nativeBuildInputs = nativeBuildInputs ++ [ final.pkg-config ];
          # propagatedBuildInputs, not buildInputs: installed headers like
          # display/GlTexture.hpp include GL/gl.h directly, so any consumer that includes
          # lvr2 headers at all -- not just code that calls into these classes -- needs it.
          propagatedBuildInputs = propagatedBuildInputs ++ [
            final.libGL
            final.libGLU
          ];
          cmakeFlags = cmakeFlags ++ [ "-DLVR2_BUILD_TOOLS=OFF" ];
          # lvr2_largescale_reconstruct is add_subdirectory()'d unconditionally on
          # !MSVC -- LVR2_BUILD_TOOLS does not gate it like the other tools -- and its
          # LargeScaleReconstruction.tcc #includes <mpi.h> outright. Real MPI isn't an
          # option here either: as soon as find_package(MPI) succeeds anywhere in this
          # CMakeLists.txt, it demands a boost_mpi component nixpkgs' boost doesn't build
          # (see the lvr2-config.cmake fix above). We don't need this tool at all -- mesh_map
          # only links the core library -- so drop it from the build entirely.
          postPatch =
            postPatch
            + ''
              sed -i '/add_subdirectory(src\/tools\/lvr2_largescale_reconstruct)/d' CMakeLists.txt
            '';
          postInstall =
            postInstall
            + ''
              sed -i '/find_dependency(MPI)/d' "$out/lib/cmake/lvr2/lvr2-config.cmake"
            '';
        }
      );
      lidarslam = patchLidarSlamLicense rosPrev.lidarslam;
      scanmatcher = patchLidarSlamLicense (
        rosPrev.scanmatcher.overrideAttrs (
          {
            propagatedBuildInputs ? [ ],
            checkInputs ? [ ],
            ...
          }:
          let
            testPackages = [
              rosFinal.ament-lint-auto
              rosFinal.ament-lint-common
            ];
          in
          {
            propagatedBuildInputs = prev.lib.subtractLists testPackages propagatedBuildInputs;
            checkInputs = checkInputs ++ testPackages;
          }
        )
      );
    };

in
{
  rosPackages = prev.rosPackages // {
    # we need to use overrideScope and an overlay to apply the changes
    # so that they propagate properly
    ${rosDistro} = prev.rosPackages.${rosDistro}.overrideScope rosOverlay;
  };
}
