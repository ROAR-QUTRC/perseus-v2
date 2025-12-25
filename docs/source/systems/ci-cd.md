# CI/CD

## What is CI/CD?

CI/CD (which stands for Continuous Integration, Continuous Delivery/Deployment) is, in theory, exactly what it says on the tin.

### Continuous Integration

Continuous Integration refers to frequently (and automatically) uploading and merging code to the main repository.
This keeps code merges small (reducing the chances and sizes of merge conflicts), and therefore more manageable.
However, that's only one part of code _integration_ - the other is continuous and _automated_ testing.
CI/CD is only made possible by continuous and automated builds and tests which run every time there is code pushed to the main repository.

#### CI testing frameworks

As this is a ROS2 project with C++ and Python nodes, the GoogleTest (gtest) and pytest frameworks are used respectively to create tests that are automatically run.

### Continuous Delivery

Continuous Delivery refers to the project always being _delivered_ in a functional, ready-to-go state, and handles any final stages needed to package the project and get it into a _deployment_ ready state.
Since this project is built with Nix, all dependencies fixed and known ahead-of-time, so there's nothing to do here.

### Continuous Deployment

Continuous Deployment is exactly what it sounds like - automatically _deploying_ a project to production after the continuous _delivery_ process of the pipeline finishes its build.
For this project, we aren't employing continuous deployment for anything but the docs website - see the [documentation system](project:/systems/documentation.md) for details on that.

## Execution

The CI/CD pipeline for this project is run entirely using [GitHub Actions](https://docs.github.com/en/actions).
The typical workflow looks something like this:

1. Check out the repo with [`actions/checkout`](https://github.com/actions/checkout)
2. Install Nix with [`nixbuild/nix-quick-install-action`](https://github.com/nixbuild/nix-quick-install-action)
3. Set up Nix output caching with [`DeterminateSystems/magic-nix-cache-action`](https://github.com/DeterminateSystems/magic-nix-cache-action)
4. (Optionally) Run some kind of update with `nix run` and commit it back to the repo
5. Run `nix build -L` (or `check`) on the output - adding the `-L` flag enables logging into the shell and makes it easier to debug when things go wrong
6. If that succeeds, the builds are passing!
7. (Optionally) Upload to cachix using one of the scripts in `software/scripts` (wrapped for execution with `nix run`)

If you're curious about any specific workflow, they're all well commented.
