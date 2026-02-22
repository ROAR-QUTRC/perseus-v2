# ROS Package Patches and Overrides

This document describes the patches and custom package builds maintained in `patches.nix` for the Perseus project.

## Table of Contents

- [PCL](#pcl)
- [Other Patches](#other-patches)

---

## PCL

The nixpkgs version of PCL (Point Cloud Library) has been updated to run on QT6, which conflicts with everything else in our environment that run on QT5. To fix this, we simply replace the QT6 inputs with QT5.

## Other Patches

(Documentation for other patches in `patches.nix` can be added here)
