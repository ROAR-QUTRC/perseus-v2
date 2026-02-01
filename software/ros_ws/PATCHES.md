# ROS Package Patches and Overrides

This document describes the patches and custom package builds maintained in `patches.nix` for the Perseus project.

## Table of Contents

- [Python QT Binding](#python-qt-binding)
- [NAV2 MPPI Controller](#nav2-mppi-controller)
- [ROS GZ Sim](#ros-gz-sim)
- [GZ Msgs Vendor](#gz-msgs-vendor)
- [Other GZ Vendors](#other-gz-vendors)
- [PCL](#pcl)
- [Other Patches](#other-patches)

---

## Python QT Binding

Python QT Binding has shifted to focus only on QT6, however all of ROS2 Jazzy (e.g. rqtgraph) still relies on QT5. The two options for QT bindings are Pyside (shiboken) and PyQt (SIP). Shiboken2 and SIP4 both work for QT5, but their updated versions, Shiboken6 and SIP6 only work for QT6. SIP4 doesn't work with Nix - it tries to edit something in the nix store when it shouldn't, so Shiboken is added to replace PyQt6 and Pyside6 in the Python QT Binding package. Pyside2 also needed a patch to allow it to work with python 3.13 and python 3.12, so it's patched in `packages/pyside2`.

## NAV2 MPPI Controller

This package depends on the _stable_ version of XTensor, but the version in nixpkgs is unstable, so XTensor and it's dependency XTL had to have their versions changed.

## ROS GZ Sim

The version in the overlay (currently 1.0.19) is incompatible with the version of gz-sim (provided by gz-sim-vendor, currently 8.9.0) that's in the overlay. The updated version had some restructuring that throws error, so we roll back two versions to 1.0.17.

## GZ Msgs Vendor

Similar to ROS GZ Sim, the version of gz-msgs (provided by gz-msgs-vendor, currently 10.3.2) is incompatible with the protobuf versions in nixpkgs (they removed protobuf_28 because it was a 'leaf package' see https://github.com/NixOS/nixpkgs/pull/416677) so we override the version and hash, accessing the underlying function and calling version 28.

## Other GZ Vendors

Because of the way ament-cmake-vendor-package-wrapped is set up, it can change the version of the package that it supplied to the vendor. However, the vendor searches for a specific version, causing version mismatches when building. Each vendor needs a patch to change the version they're searching for to the version they're given.

## PCL

The nixpkgs version of PCL (Point Cloud Library) has been updated to run on QT6, which conflicts with everything else in our environment that run on QT5. To fix this, we simply replace the QT6 inputs with QT5.

## Other Patches

(Documentation for other patches in `patches.nix` can be added here)
