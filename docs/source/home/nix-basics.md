# Nix Basics

As you've almost certainly read by now in at least one place, this project is built using [Nix](https://nixos.org/), allowing for fully declarative package and environment setup.
Whilst this provides massive advantages for reproducibility and ease of environment setup, that doesn't help if you don't know how to use it - which this document attempts to remedy.

## Nix, nixlang, and nixpkgs?

Unfortunately, the name Nix has been used to refer to multiple things.
The first is Nix the command-line tool - this is how you get it to do things.
The next is [`nixpkgs`](https://github.com/NixOS/nixpkgs) - this is a GitHub repository containing the packaging instructions for every package available by default in Nix.
Then we come to the confusing part - the Nix _{term}`language<nixlang>`_, Nix _{term}`files<Nix file>`_, and Nix _{term}`expressions<Nix expression>`_.
To simplify matters, the Nix language will be referred to as nixlang after this point.
Nix _files_ are simply files with a `.nix` extension containing a Nix _expression_.
Finally, Nix _expressions_ are simply pieces of nixlang code which can be evaluated.

## Basic Usage

In this project, you will almost always be using one of a few commands: `nix build`, `nix develop`, `nix run`, and `nix shell`.
They all share similar syntax, because they operate on the same data, just in different ways.
Since Nix is _{term}`declarative<declarative programming>`_ and _configuration-based_, you need to tell it _where_ that configuration it should act upon is.
This is the next argument after the command to run, and should point to the root of this repository (or one of its subdirectories) [^nix-search].
For example, if your current working directory is the repository root, and you run `nix build .` [^shell-dot], it will build the _default_ output of this repository - which is the workspace with everything built and ready to run.
If you're located in the _parent directory_ of the repository, you would instead run `nix build ./perseus-v2`.
:::{tip}
The leading `./` (or `../`) in this case is important!
Without it, Nix will try to resolve the location by searching through a list of predefined locations, which typically isn't what you want (except for the `nix shell` command).
:::

[^shell-dot]: `.` in the context of paths just means "the current directory", and `..` means "the parent directory".

[^nix-search]:
    Technically, it needs to point to a directory containing a `flake.nix` file (or one of its subdirectories - it'll search upwards), and since this project's `flake.nix` is in the repository root, that's why it needs to point there.
    Since it searches upwards, running `nix COMMAND .#blah` works no matter what directory you're in, as long as you're inside the repo.

However, both of those examples are using the _default output_ of the project's [flake file](#nix-flakes).
If you want to select a _different_ output, you append a `#` symbol and the {term}`attribute<Nix attribute>` to select to the path to the repo.
Again assuming you're in the repo root, if you wanted to build the project documentation instead, you would run `nix build .#docs`.

:::{tip}
For any `nix` command (and particularly `nix build`), if you want to see logs in real time instead of running the `nix log` command after the fact, add the `-L` flag.
Just beware that this will output logs for _every_ step of the process, so if you're doing a major (re-)build, you may get a lot of logs from dependencies which you don't necessarily want to see.
:::
:::{tip}
If you're just running `nix build .` or `nix build .#something`, you can actually exclude the `.` specifying the current directory, since that's where Nix searches first.
It's just been included in this documentation to make things clearer.
The `nix run` and `nix develop` commands have slightly different semantics if you do this - they'll search through output packages first, instead of dev shells or applications, which isn't normally what you want.
:::

### The `build` command

The `build` Nix sub-command is used whenever you want to, well, _build_ an output.
Nix will run everything to build the output, and if everything succeeds, it'll generate a `result` symlink [^store-immutability] in the directory where you ran the command.
That symlink points to the output of the build, which could be either a file or a directory (but is usually a directory).
Any {term}`Nix derivation` can be built using this command.

[^store-immutability]:
    Nix has the concept of a _store_ (located at `/nix/store/`), which is where it keeps everything.
    This store is _immutable_, meaning it can't be changed, and the `result` symlink points to something in here.
    Everything Nix builds (or downloads) ends up in the store, and making liberal use of symlinks to files here is how Nix keeps everything isolated.
    This does mean that store sizes can end up quite large if you don't run garbage collection, though.

### The `develop` command

This does pretty much what you think it does - it drops you into a development environment.
Much like the `build` command, you give the `develop` command a derivation to target - it will first search through the local flake's development shell outputs (this allows customising the development shells, like is done in this project - otherwise it's what you need to build the package and _only_ what you need to build the package), before searching through the ordinary package outputs.
It will then drop you into a Bash shell with all the build dependencies available and requisite environment variables set [^env-variables].
You can also configure commands to be run on entry, which allows you to do things like print out status information, or set up shell autocomplete.

[^env-variables]:
    It makes packages available by appending them to your shell `PATH`.
    When your shell looks for an executable, it searches through all the directories in your `PATH`, so by appending (or prefixing) directories to it, Nix can make programs available.

:::{note}
This is what `direnv` is configured to use for this particular project - it creates a `nix develop` shell, records the changes made to environment variables, then applies those changes to your shell environment when you change directory into this project.
Unfortunately, it _only_ transfers environment variable changes, not shell commands, which is why autocomplete is sometimes broken in a `direnv`-powered dev shell, but not in a true `nix develop` shell.
:::

### The `run` command

Next up: The `nix run` command does pretty much what you'd expect it to - it runs a program!
If there's an application defined for the output name you give, it'll run that.
Otherwise, it'll search through the normal package outputs for something by that name and will attempt to run it instead.
In this case, it will only succeed if the package outputs an executable file of the same name as the package to `$out/bin/`, or it explicitly defines which program is the main one.
You can pass arguments to the executed application after a `--` in the `nix run` command, like so: `nix run .#example-program -- --argument-to-program another_program_argument`[^nix-run-args].

[^nix-run-args]:
    If the arguments to be passed don't start with the double dash, you can actually omit the separating `--`.
    For example, you could run `nix run .#docs.decompress result/docs.7z decompressed-docs/` and everything after the derivation to execute will be passed in as arguments to the executable built by the derivation `.#docs.decompress`.

### The `shell` command

The `nix shell` command is in some ways similar to the `nix develop` command - it drops you into a shell with the specified packages.
It takes any number of arguments, each of which is a package which will be available.
For example, if you ran `nix shell .#rosCore .#pkgs.net-can-server`, you would have the outputs from `rosCore` and `pkgs.net-can-server` from the current directory's flake available in your shell.

### Nix command targets

All the `nix` commands documented above take what Nix calls "[installables](https://nix.dev/manual/nix/latest/command-ref/new-cli/nix#installables)" as their "targets".
In most cases, this is a flake output from the local directory, but it can also be something selected from any location which contains a [`flake.nix`](#nix-flakes) file.
You can use a filesystem path (like `.` or `~/perseus-v2`), `nixpkgs` (or a branch), or even fetch a repository from GitHub (like `github:github:wentasah/ros2nix`).

:::{note}
The `nixpkgs` input is special syntax, since normally you'd have to enter the full path `github:NixOS/nixpkgs`.
You can search for what's available in the [official search](https://search.nixos.org/packages) (we're using the unstable branch, but most of the time that's only minor version number changes), but the results from [MyNixOS](https://mynixos.com/) can sometimes be better.
Both searches show the executables available in a package, though sometimes the MyNixOS search doesn't manage to pick them all up.
If you can't find something, try searching both of them.
:::
::::{tip}
You can use the syntax `location/BRANCH[#PACKAGE]` to use a flake from a specific git branch, tag, or commit if the flake was fetched from a Git repository.
This is particularly useful to select the latest version from `nixpkgs-unstable`.
:::{example}
`nixpkgs/nixpkgs-unstable#uv` would select the latest version available in `nixpkgs` of the `uv` package.
:::
::::

Every [architecture](project:/architecture.md) document contains a "Nix outputs" section if applicable detailing the available outputs to use.

:::{tip}
You can override the default search behaviour of the commands if you wish.
Although `nix build` searches for packages, you can build a dev shell with `nix build .#devShells.NAME`, and an application with `nix build .#applications.NAME`.
You can also use the same syntax with the other commands to, for example, run a package directly instead of an application with the same name.
:::

## Nix Flakes

A {term}`Nix flake` is a way of writing Nix expressions which has a standard syntax, as well as providing a way to lock inputs to specific versions, enhancing reproducibility.
Whilst you can read the [wiki](https://nixos.wiki/wiki/Flakes) if you're interested in the exact syntax, the basics of what it is and its most important outputs are covered here.
The first thing you need to know is that it's how you interact with almost all of the `nix SUBCOMMAND` commands - most of them target either a flake's inputs or outputs.
Its outputs are most commonly either {term}`packages<Nix package>`, development shells, or applications.
:::{tip}
Flakes can define a default output for any of their major output types, which will be targeted if no name is given.
For example, `nix build .` can be rewritten as `nix build .#default`.
:::

### Packages

A {term}`Nix package` is most commonly used to build binary (that is, program) outputs.
However, it can also contain other packages (using what are called `passthru` attributes).
For example, in this project, although `tools` is a package in the flake, it also contains other packages (like `tools.treefmt-write-config`).
These are what the `nix build` and `nix shell` commands target.

### Development shells

These are what `nix develop` attempts to target first, before using the build environment of the package outputs.
This allows defining custom "packages" with custom build environments, so you can fine-tune your dev shell environment.
:::{note}
This very project does this!
Without it, not all ROS2 package dependencies would be available in the dev shell.
:::

### Applications

Applications are what `nix run` attempts to target first, before falling back to standard packages.
They're effectively pre-packaged commands/shell scripts which get run in your shell.

## Nixlang basics

:::{attention}
If you're not writing Nix code or digging into how this project is set up, you can skip this section.
:::

You can mostly think of nixlang like "JSON with functions".
It has all the usual types: ints, floating-point numbers, strings, lists (arrays), and sets (like a JSON object - named key-value pairs), as well as first-class support for file paths (relative to the directory of the file the path is in), as well as functions.
Most of the complexity of Nix isn't in the language itself, but rather in the tooling and conventions that have been built up around it.

### Basic Syntax

Expressions end with semicolons (`;`).

### Examples

If you're curious about anything, check out the [Nix manual](https://nix.dev/manual/nix/latest/introduction).

#### Comments

Comments in nix start with a hash (`#`) symbol.
Unfortunately, there is no option for multi-line comments at this time.

```{code-block} nix
# This is a comment
```

#### Numbers

Numbers work exactly how you expect.
They support all the standard mathematical operators: `+`, `-`, `*`, and `/`.

##### Integers

```{code-block} nix
123
```

##### Floating-point numbers

```{code-block} nix
12.345
```

#### Strings

They are normally defined using double quotes (`"`), with the standard rules about escaping special characters with backslashes (`\`).
However, due to string interpolation, you must also escape the dollar (`$`) character with a backslash as well.

```{code-block} nix
"foo bar baz"
```

However, for multi-line strings, you can use two single quotes (`''`):

```{code-block} nix
# The newlines in this string will be preserved
''
    foo
    bar
    baz
''
```

See the relevant [section](https://nix.dev/manual/nix/latest/language/string-literals.html) of the Nix manual for escaping characters here.

##### String substitution

In Nix, you can perform string interpolation with the following syntax:

```{code-block} nix
foo = "test";
bar = "${foo} baz";
```

:::{important}
The previous example, and any other examples which define multiple variables are not valid standalone Nix code.
They would need to be part of a set or a let-in block to be valid, since a Nix file can only output a single expression.
:::

In the above example, the contents of the string `bar` would be interpolated, and its final value would be `"test baz"` after the insertion of the string `foo`.
This syntax can also be used when selecting or defining attributes of a set:

```{code-block} nix
foo = {
    bar = "baz";
};
str = "bar";
quux = foo.${str};
```

For the above example, the variable `quux` would be assigned the value of `foo.bar` - in this case the string `"baz"`.

##### String concatenation

Like most other languages, Nix supports using `+` to concatenate strings.

% TODO: Paths

#### Functions

Functions are defined with the following syntax:

```{code-block} nix
input: output;
```

Where `output` is some Nix expression which may depend on `input`.
In Nix, functions can only have a single parameter - which can quickly become a problem.
There exists a solution to this, however: Create a function which returns a function!
This pattern is called [currying](https://en.wikipedia.org/wiki/Currying):

```{code-block} nix
# The brackets here are optional and normally omitted
parameter-1: (parameter-2: output);
```

Here, `output` can depend on both `parameter-1` _and_ `parameter-2`.

Functions are called with the syntax `name argument`:

```{code-block} nix
double = input: input * 2;
number = 123;
doubled-number = double number;
```

Since brackets aren't required, this can quickly get confusing.

```{code-block} nix
# Use currying to define a function with multiple arguments
fancy-operation = a: b: (2 * a) + b;
result = (fancy-operation 2) 5;
```

In the above code, the function `fancy-operation` is defined, which takes a parameter `a` and returns another function.
When evaluated in the above example, it would return the following function:

```{code-block} nix
b: (2 * 2) + b;
```

Which is then also called with the argument `5`, resulting in the output `9`.
As with defining the function, the brackets to show the order of operations are optional when calling it, and the following would be syntactically equivalent, although much more confusing (and unfortunately, more common):

```{code-block} nix
result = fancy-operation 2 5;
```

## Glossary

{.glossary}
Nix file
: A file ending with `.nix` containing a {term}`Nix expression`.

Nix expression
: A piece of {term}`nixlang` code.

nixlang
: The language (also called Nix) which the Nix _program_ evaluates.

derivation
: See {term}`Nix derivation`.

Nix derivation
: A Nix _build task_ containing all instructions and information to build a {term}`Nix package`.

declarative programming
: Declarative programming is a style of programming where everything you _want to happen_ is described, not _how to do it_.
This is in contrast to {term}`imperative programming`.

imperative programming
: Imperative programming is what you're most likely used to.
In this style, you describe a _sequence of commands_ which you want to be executed in that order.
See also {term}`declarative programming`.

Nix flake
: A Nix flake is a standardised way of writing a Nix expression, which also has dependencies version-pinned in a lockfile.

Nix attribute
: A member of a Nix set.

    For example, the following set contains the attributes `foo` and `bar`: `{ foo = "baz"; bar = "quux"; }`

Nix package
: The output of a {term}`Nix derivation`.
Most commonly binary (program) outputs under `$out/bin/`, as well as library outputs under `$out/lib/` and man pages.
