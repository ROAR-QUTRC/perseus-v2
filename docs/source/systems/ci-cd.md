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
Since this project is built with Pixi, with all dependencies pinned in `pixi.lock`, there's nothing extra to do here.

### Continuous Deployment

Continuous Deployment is exactly what it sounds like - automatically _deploying_ a project to production after the continuous _delivery_ process of the pipeline finishes its build.
For this project, we aren't employing continuous deployment for anything but the docs website - see the [documentation system](project:/systems/documentation.md) for details on that.

## Execution

The CI/CD pipeline for this project is run entirely using [GitHub Actions](https://docs.github.com/en/actions).
The typical workflow looks something like this:

```{graphviz}
:caption: Per-push CI pipeline, end-to-end
:align: center

digraph cicd {
    graph [rankdir=LR, bgcolor="transparent", fontname="Roboto",
           nodesep=0.3, ranksep=0.45];
    node  [fontname="Roboto", fontsize=10, style="filled,rounded",
           shape=box, penwidth=1.1, margin="0.18,0.10"];
    edge  [fontname="Roboto", fontsize=9, color="#7a6cad"];

    trigger [label="git push\n/ pull_request", shape=cds, fillcolor="#ec407a", fontcolor="white"];

    subgraph cluster_run {
        label=<<b>GitHub Actions runner(s)</b>>; labeljust="l";
        style="rounded,filled"; color="#3949ab"; fillcolor="#1a237e"; fontcolor="#d6c8ff";

        checkout [label="actions/checkout", fillcolor="#311b92", fontcolor="white"];
        setup_pixi [label="prefix-dev/setup-pixi\n(installs Pixi, restores cache)", shape=cylinder, fillcolor="#0277bd", fontcolor="white"];
        format [label="format job:\npixi run -e format fmt-check", fillcolor="#5e35b1", fontcolor="white"];
        build [label="build job (matrix):\nlinux-64 + linux-aarch64\npixi run -e default build-test / test", fillcolor="#5e35b1", fontcolor="white", penwidth=2.0];
        build_sim [label="build-sim job:\nlinux-64 only\npixi run -e simulation build-test / test", fillcolor="#5e35b1", fontcolor="white"];
    }

    pass [label="✓ green build", shape=oval, fillcolor="#2e7d32", fontcolor="white"];
    fail [label="✗ failure", shape=oval, fillcolor="#b71c1c", fontcolor="white"];

    trigger    -> checkout;
    checkout   -> setup_pixi;
    setup_pixi -> format;
    setup_pixi -> build;
    setup_pixi -> build_sim;
    format     -> pass;
    build      -> pass;
    build_sim  -> pass;
    format     -> fail [label="non-zero exit", color="#b71c1c"];
    build      -> fail [label="non-zero exit", color="#b71c1c"];
    build_sim  -> fail [label="non-zero exit", color="#b71c1c"];
}
```

1. Check out the repo with [`actions/checkout`](https://github.com/actions/checkout)
2. Install and cache Pixi with [`prefix-dev/setup-pixi`](https://github.com/prefix-dev/setup-pixi)
3. Three jobs run (in parallel, each on its own runner):
   - **format**: `pixi run -e format fmt-check` (treefmt, over the whole repo)
   - **build**: a matrix over `ubuntu-latest` (linux-64) and `ubuntu-24.04-arm` (linux-aarch64) running `pixi run -e default build-test` then `pixi run -e default test`
   - **build-sim**: linux-64 only, running `pixi run -e simulation build-test` then `pixi run -e simulation test`
4. If all three jobs succeed, the build is passing!

If you're curious about any specific workflow, they're all well commented - see `.github/workflows/all.yaml`.
