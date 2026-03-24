# Flakes

Flakes are an experimental feature of Nix, which means they can be subject to
breaking changes. Despite this, they introduce a few concepts that greatly
increase reproducability, so they have been widely adopted by the nix community.

## Breakdown of key concepts

Flakes, at their core, are nix functions that take an attribute set of inputs,
and define a set of outputs based on these inputs.

### Inputs

Each input contains a url to a repository containing a flake (except for special
cases we won't go into because we aren't using them), and optional overrides for
the flake's inputs. Overriding the inputs is useful because each imported flake
likely contains a copy of nixpkgs (or some other repo) that other inputs also
contain. To avoid downloading multiple copies of these dependencies, each at
different versions (commit hashes), the duplicate inputs are set to 'follow' the
input of another flake input.

### Outputs

The outputs attribute of a flake contains multiple possible options, each with
their own command to access the option. For example, `nix develop` searches the
flake for the `devShells` attribute. See the following table for what each
command does and the attribute/s it searches for.

| Command       | Description                                                                                                                                                                                                                                                                                            | Attribute/s             |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------- |
| `nix fmt`     | Runs a formatter on the root directory (the one containing the flake.nix) and all subdirectories                                                                                                                                                                                                       | `formatter`             |
| `nix run`     | Runs a program in your shell. If the target is not found in `apps`, the `packages` will be searched, and the program defined in `package_output/bin/package_name` will be run                                                                                                                          | `apps`, `packages`      |
| `nix develop` | Places you into an environment containing all the packages defined in the `devShell` attribute. If the target is not found in `devShells`, `packages` will be searched, and the environment will contain everything needed to build the package, including environment variables set in the derivation | `devShells`, `packages` |
| `nix build`   | Build a package - the output of the build will be placed in `./result`                                                                                                                                                                                                                                 | `packages`              |
| `nix shell`   | Places you into a shell with the target package, so you are able to use the program as if it were installed                                                                                                                                                                                            | `packages`              |
