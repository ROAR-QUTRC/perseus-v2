# Scripts

## clean.sh

Removes any built, installed, or generated files (`result`, `build`, `log`, `install`, `generated` directories anywhere in the repo) and basically resets the repo back to when it was first installed. Run it directly: `./software/scripts/clean.sh`.

## cuda-test.sh

Checks the status of the Pixi-managed CUDA development environment (`machine-learning` env): whether `nvcc` is available and Pixi-managed, whether a GPU is detected via `nvidia-smi`, and runs a small compile+run smoke test if a GPU is present.

## cuda-demo.sh

A visual side-by-side CPU vs CUDA matrix-multiplication demo, for showing off the `machine-learning` env's GPU acceleration. See `--help` for options (matrix size, sequential vs race mode).

## member-setup.sh

This script should be run whenever installing the repo on a fresh device. It installs required packages (`direnv`, `git`, `gh`), clones the repo, installs Pixi if it isn't already present, runs `pixi install` and `pixi run -e default build`, then restarts the shell. It is designed to be run by the command:

```console
curl https://raw.githubusercontent.com/DingoOz/perseus-lite/refs/heads/main/software/scripts/member-setup.sh | bash
```

Which gets the raw content of the .sh file and executes it by piping it directly into bash.

## pixi-env-sanitize.sh

Wired into `pixi.toml`'s `[activation] scripts`, so it runs automatically on every `pixi shell`/`pixi run`. It strips path entries pointing at a native system ROS install (`/opt/ros/*`) or a foreign colcon workspace from `PATH`-like environment variables, defending against contamination leaking in from a parent shell that has already sourced a different ROS/Gazebo installation. See `ERRORS.md` and the "System-ROS contamination hazard" note in the repo's `CLAUDE.md` for the failure mode this prevents.
