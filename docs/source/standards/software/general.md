# General Software Standards

## General Naming

Almost everything in [Naming Things in Code](https://www.youtube.com/watch?v=-J3wNP6u5YU) applies to ROAR code. I highly recommend checking out _all_ of his ([CodeAesthetic's](https://www.youtube.com/@CodeAesthetic/videos)) videos - they're all excellent, and almost everything is applicable here. They explain many of the _principles_ behind why certain rules are in this document.

The one exception to this video is abbreviations, as documented below.

Use `snake_case` for variable and function names, unless otherwise stated by a language standard.

### Spelling

Use American spelling for names - color instead of colour, serialize instead of serialise, etc. This is because most pre-existing code uses American spelling, and this will keep consistency with it.

### Abbreviations

Use minimal abbreviations. Some things are either extremely common, such as the use of `i` as an index variable in a `for` loop, or language suggested such as `it` for `iterator`, but overall if you're abbreviating something, you're probably making a mistake.

#### Acceptable

```cpp
speed_settings_t current_speeds;
```

#### Incorrect

```cpp
speed_settings_t cur_speeds;
speed_settings_t cur_spds;
```

#### Exceptions

Below is a list of some of the common exceptions to this rule.

| Abbreviation | Full name    | Note                                                                                                    |
| ------------ | ------------ | ------------------------------------------------------------------------------------------------------- |
| `i`          | `index`      |                                                                                                         |
| `idx`        | `index`      |                                                                                                         |
| `it`         | `iterator`   | Language standard (C++)                                                                                 |
| `pos`        | `position`   |                                                                                                         |
| `tmp`/`temp` | `temporary`  | Only to be used as prefixes. Also should not be used around temperatures as that gets confusing quickly |
| `max`        | `maximum`    |                                                                                                         |
| `min`        | `minimum`    |                                                                                                         |
| `prev`       | `previous`   |                                                                                                         |
| `init`       | `initialize` |                                                                                                         |

### Units in variable names

Units in variable names should be avoided, and a type which specifies or eliminates the units should be used instead. However, if this is unavoidable or impractical, units are exempted from abbreviation rules.

### Descriptiveness

Variable names should be descriptive and describe what they do/are.

#### Acceptable

```cpp
speed_settings_t target_speeds;
```

#### Incorrect

```cpp
speed_settings_t tmp; // bad! Both abbreviated and not descriptive
speed_settings_t speeds; // what speeds?
speed_settings_t target; // target what?
```

### Booleans

Boolean (true/false) variables should be unambiguous in their naming such that you know exactly what it represents by looking at it.
In most cases, this means that they should have a verb prefix indicating that they are a boolean, and what it does.

#### Acceptable

```cpp
bool is_valid;
bool should_exit;
bool has_settings;
```

#### Incorrect

```cpp
bool valid;
bool exit;
bool settings;
```

However, in some cases (such as as members of an appropriately named `struct`), it may be appropriate to omit a prefix.
For example, the following code is acceptable as they are part of a status flag register struct:

```cpp
struct fet_status_t
{
    bool alert_asserted;
    bool ddsg_asserted;
    bool dcht_asserted;
    bool pre_discharge_fet_on;
    bool discharge_fet_on;
    bool pre_charge_fet_on;
    bool charge_fet_on;
};
```

### Constants

Regardless of the language, constants should be named with `SCREAMING_SNAKE_CASE` and, if possible, marked as constant/immutable.

:::{important}
In, C++, this means using `constexpr`, or `const` if you can't use `constexpr`.
Python has no way of doing this.
Nix, being a functional programming language, does not allow "variable" modification to start with.
:::

Constants should not be prefixed with types or any information other than their name.

## Class Methods and Variables

### Class method names

Method names should describe the _action_ that they perform, concisely and unambiguously. They should generally be in verb\[Noun\] format.

#### Acceptable

```cpp
class Settings
{
public:
    int get_max_speed();
    void set_max_speed(int max_speed);

    // note that write() may also be acceptable here depending on the context
    void write_to_disk();
};
```

### Visibility

All member variables inside a class should be private. If code outside the class needs access to the variables, write accessors and mutators. Specify _all_ visibility - in C++ classes default to `private` visibility and structs to `public`, but as structs should contain no functionality, this is a nonissue for them.

#### Correct

```cpp
class Settings
{
public:
    int get_max_speed();
    void set_max_speed(int max_speed);

    // note that write() may also be acceptable here depending on the context
    void write_to_disk();
private:
    int _max_speed;
};
```

### Member Ordering

Methods and variables inside a class should be ordered as follows.
Constants have been excluded from the list, as it may make sense to place them before _or_ after type definitions under different circumstances.
However, either way, they should come before any functions are defined.

#### Public

- Types
- Constructors and destructor
- Handler functions
- Accessors/mutators (preferably paired if applicable)
- Other functions

#### Private

- Types
- Internal methods
- Variables

#### Correct

```cpp
class Settings
{
public:
    typedef int rpm_t; // this is just an example, doing a typedef for this is unnecessary

    Settings();

    int get_max_speed();
    void set_max_speed(int max_speed);

    void write();
private:
    rpm_t _calculate_max_rpm();

    int _max_speed;
};
```

### Private Methods and Variables

All private members and variables of a class should be prefixed with an underscore (`_`).

#### Acceptable

```cpp
class Settings
{
    // ...
private:
    void _load_defaults();

    int _max_speed = 0;
};
```

### Mutators and Accessors

If a getter or setter for a variable is written, they must be prefixed with `get` or `set`, and the name of the function should be the same as that of the internal variable it accesses.

When accessing aggregate classes (lists, maps, etc.), the function should be prefixed with `add`, `get`, `remove`, or `set` (depending on the type of the aggregate, other prefixes may be appropriate), and the plurality of the variable modified as needed (such as `get_configuration` accessing `_configurations`).

Note that for some classes, it may not make sense for them to be mutable, and in these cases, providing only getters is perfectly acceptable.

#### Acceptable

```cpp
class Settings
{
public:
    // both referencing _max_speed
    int get_max_speed();
    void set_max_speed(int max_speed);

    // adds a member to the _speed_configurations member
    void add_speed_configuration(speed_settings_t configuration);

    // gets a member from the _speed_configurations member
    void get_speed_configuration(size_t idx);
};
```

## Scoping

### Globals

There should be no project-wide global variables (constants are not _variable_ so use them as needed, though namespaces are highly recommended). Use functions to interact between modules.

### Class data members

All data members should be private, except in behaviour-less aggregates (C-style structs).

## Files

### Naming

Files should be named with `snake_case`. If there is a pre-existing standard for the file names, follow that. In the case of code, the filename should describe the code within it. For other files, the filename should describe its contents. The filename of date-specific data should start with the ISO 8601 date that the data was recorded (eg `2024-08-01_can_dump`).

### Single Responsibility Principle

Every class/module/library should only have a _single responsibility_. If you're maintaining otherwise disparate code together, that's a sign that you may need to refactor and split that code out.

### Modules

Each file should be a self-contained _module_ as far as possible, interacting with other modules through the API that they expose. The file name (both header and source) should be the name of the module - such as a module named `ui` handling core UI code. Making folders to keep module code organised is also acceptable.

### Extensions

C source files should end with `.c`, C header files with `.h`, C++ source files with `.cpp`, C++ header files with `.hpp`, and Python files with `.py`. Whilst using the `.h` extension will work for C++ headers, `.hpp` specifically denotes that it is a C++ header and is thus used in this project. Additionally, the `.cpp` extension is used for C++ source files over `.cc` - it's clearer what the file is and is much more common.

#### Correct

```text
.
├── include
│  └── settings.hpp
└── src
   └── settings.cpp
```

#### Incorrect

```text
.
├── include
│  └── settings.h
└── src
   └── settings.cc
```

## Types

### Enums

Any `enum` _types_ should be named in `snake_case`, and the actual `enum` _values_ should be named in `SCREAMING_SNAKE_CASE` since they are constants.

C/C++ `enum`s should have a prefix containing the type of the `enum` (see the example), but C++ `enum class`es (or Python `enum`s) should not be prefixed, as they are accessed through the base class.

#### Acceptable

```cpp
enum speed_options
{
    SPEED_SLOW = 0,
    SPEED_NORMAL,
    SPEED_FAST,
    SPEED_MAX
};
enum class directions
{
    FORWARD = 0,
    NEUTRAL,
    REVERSE,
};
```

## Design Guidelines

### Versioning

Follow [BreakVer](https://www.taoensso.com/break-versioning). That is, versions should follow the form `major.minor.nonbreaking[-optional-qualifier]`. Nonbreaking updates should be exactly what they say - non-breaking. Updating version bumps should always be safe (and recommended) to perform. Minor updates _may_ break code in a _minor_ way - read the changelog to see if you're affected. Major updates might break code in a _major_ way - read the changes!

Whenever you perform a major (possibly breaking) change, update the major version number. Whenever you make a breaking minor change, update the minor version number. For non-breaking changes, update either the non-breaking or minor version numbers as appropriate.

Whenever a version number is incremented, all lower numbers should be reset to zero - that is, a major version bump from `1.1.1` would be `2.0.0`.

### Warnings are not acceptable

If the compiler (or linter) is complaining about something, it's almost certainly right. Fix the warnings before raising a PR to merge code onto the main branch. This not only helps to enforce a higher code quality, but also to make new (and possibly more important) warnings much more obvious when they _do_ occur.

:::{note}
The only place this doesn't apply is the documentation itself, funny enough - if there are warnings for the auto-generated Doxygen documentation, you can safely ignore those.
However, other warnings are still a problem!
:::

### Keep functions short

Short functions are easier to debug, easier to test, and easier to maintain. Long functions may be appropriate for the problem, but much of the time it will be easier to split the function up. There will be no hard limit enforced, but the OpenStack standard recommends starting to think about refactoring after 40 lines. After about 60 lines, the function is starting to get long and there's a decent chance that it should be refactored - note that refactoring should only occur if it wouldn't harm the structure of the program! If a function has better logical flow, or the problem can't be split up, then large functions are fine - but problems often _can_ be split up.

Long functions can also be hard to maintain, especially when adding features. The longer a function is, the more context gets introduced that a programmer must be aware of and work around. Making changes while not aware of that context could introduce bugs.

As a rule of thumb, if your function has more than 3 levels of indentation, have a look at it and consider if it would be made clearer by refactoring (see [Why You Shouldn't Nest Your Code](https://www.youtube.com/watch?v=CFRhGnuXG-4)). Of course there are many exceptions to this, but it often serves as a good starting point. This is because each level of indentation generally indicates a branching control flow. At more than 3 levels of indentation, there's likely a fairly large number of different control flows, all of which the end programmer has to keep in mind when modifying the code.

In short, prefer short and focused functions with a single responsibility.

### Make everything `const`

If it can be made `constexpr` or `const` (or equivalent, if not in C++), do so - both for variables and methods.
If a method doesn't modify internal data, make it `const`.
However, if it can be evaluated at compile time (such as for things like hardware-dependent constants), use `constexpr` (C++ specific).
This prevents a whole class of bugs where variables are modified which were not expected to be modified.

### Prefer composition to inheritance

See [The Flaws of Inheritance](https://www.youtube.com/watch?v=hxGOiiR9ZKg) - the TL;DW is that inheritance breaks down when you want to do something different from what the base class intended. If instead each class is separated and takes what was the parent class as an input, the "implementation overlap" is significantly reduced. Interfaces can also help with this, further reducing implementation-specific details.

### Prefer providing abstract interfaces

Interfaces should contain no implementation code - that is, all members should be _abstract_.

### Make interfaces easy to use and hard to use incorrectly

If there are "gotchas" when using the interface, then you should consider re-designing the interface so that they are instead eliminated. A common problem is that of putting too much responsibility in a single interface - classes can inherit from multiple interfaces, don't be afraid to split things up! Inheriting multiple smaller, focused interfaces is better than inheriting one over-scoped interface that we don't actually need everything in, which will cause bugs down the line.

### Prefer minimal classes over monolithic classes

Multiple smaller classes with well-defined interfaces and APIs will be easier to maintain than a single large class which has hard-to-define links between its components. If you _do_ need to define a monolithic class (sometimes good reasons do exist), then be very clear as to why you're doing so, and ensure that data flow is well defined.

### Be clear what kind of class you're writing

If it's not clear what kind of class you're writing, then that's a suggestion that perhaps your class needs to be refactored. Maybe the class itself needs to be more specific but implement a new interface, or it should be split up into multiple different classes.

### Treat class design as type design

See [Effective C++ item 19: Treat class design as type design](https://blog.ycshao.com/2012/11/23/effective-c-item-19-treat-class-design-as-type-design/). It primarily comes down to:

- Consider if you _really_ need what you're about to create
- How will it be used?
- What restrictions does it have?
- Is it useful?

### Avoid inheriting from classes that were not designed to be base classes

If a class was not _designed_ to be a base class, there are likely to be internal implementation details that inheriting from it will either cause to break, or will break your derived class.

### Public inheritance models "is a"

What this means is that public inheritance should be close to interfaces - if a class publicly inherits from another, it "is an implementation" of the base class and should behave as such. See also [composition](#prefer-composition-to-inheritance).

### Public inheritance is substitutability

Inherit, not to reuse, but to be reused. This means that, like when implementing interfaces, there should be no unexpected behaviour if using a derived class like the base class it inherits from.

### Private inheritance models "is implemented in terms of"

If a class inherits privately, it should be because it is making use of features from the base class, they do not necessarily have to have conceptual relationships.

### Don't give away your internals

Code making use of a class (or module) should not care about its internal implementation. If it does, that means that the API for that class needs to be redesigned so that internals are no longer exposed.

### Practise safe overriding

When overriding virtual functions (or equivalents in other languages), ensure that substitutability is preserved - that is, the overridden function can be used in the same contexts as the original. This means things like not changing default arguments, preserving pre- and post-conditions, and being careful not to hide overloaded functions.

### Prefer simple code over "clever" code

Unless it's actively a performance issue (in which case it should be clearly commented with proof), prefer simpler code that's easier to understand and debug over "clever" code which has small benefits.

### Use sensible defaults

All values which _can_ have default values/initialisers provided should. The default initialisers should be sensible (that is, result in a valid data type). For example, a counter of some kind in a class can be default initialised to `0` and therefore does not need to be specified in the constructor initialiser lists.

This also goes for _programs_ you write, not just functions and classes - for example, a ROS node or launch file should work "as expected" by default, with options needing to be _manually set_ to change its behaviour from "working normally".

### Avoid magic numbers

There should be no "bare" magic numbers in your code. They should be one of the following instead:

- Commented - easiest but also worst solution
- Converted to a _named_ constant
- Explained by the variable names, such as `output_revolutions_per_second = input_rpm*60;`

## Documentation

All code in the project should be documented with [Doxygen](https://www.doxygen.nl/manual/docblocks.html) comments. This is only _required_ for top-level files/classes, but is _strongly encouraged_ for the entire public API. Finally, it is preferred that as much as possible is documented. The documentation priority list is:

- File/module/class (purpose, usage, etc.)
- Public API
- Internal API/functions
- Internal types (optional if self-descriptive enough)
- Internal variables (also optional if self-descriptive)
