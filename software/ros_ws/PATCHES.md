# ROS Package Patches and Overrides

This document describes the patches and custom package builds maintained in
`patches.nix` for the Perseus project.

## Table of Contents

- [Fast-LIO](#fast-lio)
- [Other Patches](#other-patches)

---

## Fast-LIO

This package has git submodules, which can't be retrieved by fetchFromGitHub
automatically. This has to be overridden from the generated package. Setting
fetchSubmodules to true just uses fetchGit instead, so I've used fetchGit to be
cleaner. Fast-LIO also compiles specifically for c++14, which conflicts with the
rclcpp library, which is written for c++17.

## Other Patches

(Documentation for other patches in `patches.nix` can be added here)
