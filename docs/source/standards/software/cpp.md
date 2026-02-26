# C++ Standards

Many of the items covered in this document are also reiterated [here](http://micro-os-plus.github.io/develop/sutter-101/) with different wording, as well as several other things being covered. Have a look and see if it's covered in the linked page if you're confused about something, it might explain it differently.

## Header Files

### Use C++ includes over the C includes

This means using `#include <cstlib>` instead of `#include <stdlib.h>` and similar. If you're including a standard library file ending in `.h`, chances are it's a C include and you should be using the `<clibname>` alternative.

#### Correct

```cpp
#include <cstdint>
```

#### Incorrect

```cpp
#include <stdint.h> // don't do this
```

### Include types

In C++, there are two types of `#include` directive - `#include <filename.hpp>` and `#include "filename.hpp"`. The former searches for files on the compiler include path (eg STL headers, external libraries, etc) before project paths (if it searches project paths at all). Quoted include directives search the project paths first, _then_ the system include paths. As such, include directives using quotes are to be used for files within the same project, and angle bracket includes should be used for everything else. This ensures that even in the event of a naming conflict, the correct file will usually be selected. It also serves to differentiate intention - that is, _using_ a behaviour as opposed to (usually) _implementing_ a behaviour.

### Assume nothing

Make no assumptions about files being included before whatever you're writing. If your header file needs another file to be included before it, **do not** assume that it will be included. Include it yourself. If you need a certain variable to be declared, declare it. If your code can be broken by including it into a different file, or with a different include order of files, it needs to be fixed.

#### Correct

```cpp
#include <cstdint>

uint8_t adder(uint8_t input);
```

#### Incorrect

```cpp
// assumes that something includes <cstdint> before it!

uint8_t adder(uint8_t input);
```

### Don't pollute the namespace

You should never use a `using` directive in a header. This goes for both `using namespace` directives and specific symbol inclusion such as `using std::vector`.

#### Correct

```cpp
#pragma once

#include <cstdint>
#include <tuple>
#include <vector>

class Settings
{
public:
    void add_setting(uint8_t id, uint8_t value)
    {
        _ids.push_back(id);
        _values.push_back(value);
    }
    std::pair<uint8_t, uint8_t> get_setting(uint8_t index)
    {
        return std::make_pair(_ids[index], _values[index]);
    }

private:
    std::vector<uint8_t> _ids;
    std::vector<uint8_t> _values;
};
```

#### Incorrect

```cpp
#pragma once

#include <cstdint>
#include <tuple>
#include <vector>

// now everything after the inclusion of this file has to deal with the inclusion of the entire std namespace
using namespace std;
class Settings
{
public:
    void add_setting(uint8_t id, uint8_t value)
    {
        _ids.push_back(id);
        _values.push_back(value);
    }
    pair<uint8_t, uint8_t> get_setting(uint8_t index)
    {
        return make_pair(_ids[index], _values[index]);
    }

private:
    vector<uint8_t> _ids;
    vector<uint8_t> _values;
};
```

### Use include guards

Put `#pragma once` as the first line of all headers. This prevents files from being included more than once in a file. Whilst technically `#pragma once` is compiler specific, it's common enough that it's effectively a standard, and all C++ code on the rover is compiled with `gcc` anyway making it a nonissue. You may have already seen the standard include guard pattern of

```cpp
#ifndef __MY_FILE_H
#define __MY_FILE_H
// code here...
#endif // __MY_FILE_H
```

or something similar - whilst this will work the exact same way, it is liable to programmer mistakes if a file gets moved, renamed, or copied, whereas the pragma directive doesn't have these issues.

#### Correct

```cpp
#pragma once

#include <cstdint>

class Settings
{
    // ...
};
```

#### Incorrect

```cpp
#include <cstdint>

// now if this file is included twice, there'll be an error since the class is declared twice
class Settings
{
    // ...
};
```

## Files

### File System Structure

Source code files should all be located under the `src/` directory inside the project, and include files under `include/`. If the application is a program, the entry point should be located inside a file named `main.cpp`. If, as in the case of ROS nodes, this is not applicable, name the source files according to what they _are_ - this will primarily mean naming them with nouns and adjectives.

Libraries may need internal, private header files - if necessary, these should be present under a `priv_include` directory, and the builder/installer should be configured appropriately to not include them in final outputs.

## Variable Declarations and Naming Standards

### Naming Convention

Variables should be named in `snake_case`.

:::{warning}
This has changed.
Previously, the standard was to use `camelCase` and name local static variables with `snake_case` to differentiate them and indicate that they will have different behaviour.
However, to comply with de facto ROS standards, this has been changed.
:::

In addition, C++ code in this project is compiled with `-Wshadow`.
To allow for situations where you want to have a parameter in a constructor with the same name as a class member (for example, constructor initializer lists), it is acceptable to prefix the _parameter_ name with an underscore as needed.

:::{important}
This should only be needed for structs!
If you're mixing public and private member _variables_, you're probably doing something wrong.
In general, structs should have fully public member variables, and classes fully private or protected (and underscore prefixed).
With _both_ public and private variables, and the need to prefix parameters with underscores, it becomes highly unclear what variable is associated where, so try to avoid that.
:::

For example:

```cpp
struct foo_t {
    foo_t(int _bar) : bar(_bar) {}

    int bar;
};

```

### Naming Convention (Arduino-only)

The exception to the above if is you're writing firmware based on the Arduino core.
If the code you're writing is primarily Arduino and/or FreeRTOS-based, feel free to use either `camelCase` or `snake_case` - just keep whatever you do consistent internally in the project.
However, if your code significantly interacts with either ESP-IDF or this project's internal libraries, `snake_case` should be preferred as it's the standard in use for both of them.

### Variable Type Declarations

Use the format `[static] [constexpr/const/volatile] type [&/* [const]]` (in order, compile-time constant/run-time constant/volatile, type name, reference/pointer, constant pointer modifier).
Whilst OpenStack recommends putting the type first, writing it this way reads and writes the same way that you would speak the type.

### Scoping

Unless you have a _very good reason to_, all your global variables (inside a module) should be declared `static`, and all your internal functions _must_ be declared `static`. This prevents a large number of bugs where the program may refuse to compile, variables may be inadvertently modified outside of their intended use, or internal functions called externally.

This is because the C++ (and C) compiler has _global_ (project-wide) linkage by default for global variables and functions. The `static` modifier specifies that they are only valid _inside the file they're declared in_ and thus prevents them from being used in other files. Note that this only applies to **global** variables and functions!

#### Correct

##### settings.cpp

```cpp
#include "settings.hpp"

static settings_t settings;

static void load_settings_from_file(void) { };
```

#### Incorrect

##### settings.cpp

```cpp
#include "settings.hpp"

// other code can modify settings now!
settings_t settings;

// and other code can also call load_settings_from_file, which could cause *really* bad side effects since you can call an un-declared function as long as it's defined somewhere
void load_settings_from_file(void) { };
```

### Namespaces

There is very little need for namespaces in ROAR software (except for common libraries such as `hi-can-lib` and associated libs under `hi_can`). However, if you do need a namespace, keep it to all lowercase single words, and if that's impractical, multiple words of `snake_case` is also acceptable - just keep it as short as possible while remaining understandable.

#### Acceptable

```cpp
hi_can
hi_can::drive
hi_can::drive::traction_control
```

#### Incorrect

```cpp
rover_libraries::drive
RoverLibraries
```

### Class names

Class names should always be `PascalCase` and _describe_ what they do, with minimal abbreviation. They also should not be prefixed (especially not with `cls`!). Interfaces (abstract base classes) should probably be named with an adjective describing the interface, such as `Serializable`.

#### Common Pitfalls

- `BaseSettings`: If you're prepending `Base` to a class, don't. This will just confuse later programmers, and serves instead as a suggestion that you need to rename the _child_ classes.
- `ISerializable`: Just don't. Type definitions exist, you don't need to add useless prefixes.
- `Utilities`: This is bad because if you're writing a utilities class, it almost certainly means that you need to refactor those utilities out somewhere closer to where they need to be.

#### Acceptable

```cpp
class DriveController;
class Serializable;
```

#### Incorrect

```cpp
class drive_controller; // incorrect case
class SerializableData; // extraneous descriptor of "Data" added
class Utilities; // utilities for what?
class Utils; // abbreviation, and also see above
```

### Struct and typedef names

Structs and typedef types should be named using `snake_case` and suffixed with `_t`.

## Formatting

### Formatter

Use `clang-formatter` with `-style=file`. A `.clang-format` file with the following configuration is provided in the root directory of this project.

```yaml
BasedOnStyle: Google
UseTab: Never
IndentWidth: 4
TabWidth: 4
BreakBeforeBraces: Allman
AllowShortIfStatementsOnASingleLine: false
IndentCaseLabels: false
ColumnLimit: 0
AccessModifierOffset: -4
NamespaceIndentation: All
FixNamespaceComments: false
AlignConsecutiveMacros: true
```

If you're using VSCode, either set `Clang_format_style` to `file` (this is the default setting, and is preferred - it will use `.clang-format`), or to the following:

```yaml
{
  BasedOnStyle: Google,
  UseTab: Never,
  IndentWidth: 4,
  TabWidth: 4,
  BreakBeforeBraces: Allman,
  AllowShortIfStatementsOnASingleLine: false,
  IndentCaseLabels: false,
  ColumnLimit: 0,
  AccessModifierOffset: -4,
  NamespaceIndentation: All,
  FixNamespaceComments: false,
  AlignConsecutiveMacros: true,
}
```

## Class Design Guidelines

### Structs vs Classes

Structs should be used only for data storage. Adding functions to them is acceptable as long as those functions only serve to perform data conversion to/from other types or provide operators. If you need to do anything else, or need private data fields, you should be using a class instead.

### Always use RAII (or RIIA) and understand who owns what

RAII (Resource Acquisition Is Initialisation) is a practice where _allocating_ (or acquiring) a resource also _initialises_ it. This means making use of appropriate constructors, and explicitly forbidding your data from being in invalid states - creating the data _forces_ it to be in a valid state, and it cannot be invalidated until it is no longer needed. Don't be afraid to use ` =delete` to prevent a constructor or operator from being used! Just be aware of _what_ you're doing and _why_.

One of the goals of RAII is to simplify memory/resource ownership - if the memory/resource is locked/used/allocated with the acquisition of a class, then unlocked/released/freed with the destruction of that class, then resource handling is handled simply by variables going in/out of scope.

### Rule of Three

If you find yourself writing a _destructor_ (`~ClassName()`), copy constructor, or assignment operator, then you almost certainly need to adhere to the [Rule of Three](https://stackoverflow.com/questions/4172722/what-is-the-rule-of-three) (or Five) and implement all of them.

If you're needing to implement the Rule of 3/5, you're probably managing either pointers or a limited resource. To this end, you probably also need to investigate the [copy-and-swap](https://stackoverflow.com/questions/3279543/what-is-the-copy-and-swap-idiom) idiom.

### Separate inline method definitions

Don't define inline methods inside the class.

#### Correct

```cpp
class foo
{
public:
    void bar();
};

inline void foo::bar() { }
```

### Don't try to return a reference when you must return an object

References don't always behave the way you'd expect. The following code will break:

```cpp
const Point& normalize(const Point& start_point)
{
    Point normalized();
    // ... logic here
    return normalized;  // this line breaks!
}
```

This happens because the `normalized` variable goes out of scope at the end of the function and gets destroyed, making the returned reference invalid. A solution to this problem is to use smart pointers and dynamically allocate the return value - this may be a valid solution, but it is often better simply to return an object and let the compiler use the copy constructor:

```cpp
Point normalize(const Point& start_point)
{
    Point normalized();
    // ... logic here
    return normalized;
}
```

If the variable you are returning will stay alive after the function call ends (such as static variables), then it may be acceptable to do so - just be aware that references don't force an object to stay in scope.

### Avoid Global Static Class Instances

Avoid global static class instances because the initialisation order is not guaranteed. If you need a singleton object, use a public static class method named `singleton()` which initializes and returns method-scoped static object. Because the object is _static_, the reference will remain valid. Ensure the singleton class' constructor is private to avoid any other way of creating the object. You may also `delete` the constructor instead if it is not needed.

#### Acceptable

```cpp
class Singleton
{
public:
    Singleton& singleton()
    {
        static Singleton singleton;
        return singleton;
    }

private:
    Singleton();
};
```

### Don't override `new` and `delete`

There should be no need to write custom memory management operators. Almost every good reason for doing so is in the context of custom container classes (such as the STL `std::vector` and `std::list` classes).

However, if in the extremely unlikely case that multiple people have agreed that custom memory management is the best solution, follow the following:

- Always provide `new` and `delete` together
- If you provide any class-specific new, provide all of the standard forms (plain, in-place, and `nothrow`)

### Pimpl is useful, but unlikely to be necessary

Pimpl stands for "pointer to implementation". Specifically, it looks like this:

```cpp
class Example {
   //  public API here
private:
   class ExampleImpl;
   unique_ptr<ExampleImpl> _impl;
};
```

This allows private implementation details (the entire `ExampleImpl` class) to be declared and defined purely inside of source files, thus keeping private details out of header files. It can also reduce compile times, since modification to the implementation of the functionality only requires re-compiling the implementation source and re-linking. However, the advantages provided by this pattern are significantly outweighed by the hit to readability this pattern causes. It may be useful in some cases, but it is unlikely to be needed in ROAR code.

### Prefer non-member non-friend functions to member functions

This one isn't very well named. In short, it means that if a function doesn't have to have access to internals of a class, it shouldn't be part of the class.

### Declare non-member functions when type conversion should apply to all parameters

This is explained quite well [here](https://blog.ycshao.com/2017/10/16/effective-c-item-24-declare-non-member-functions-when-type-conversions-should-apply-to-all-parameters/). In short, it means that some functions (particularly operators) may need to be declared _outside_ of a class, and use that class as inputs to allow for implicit type conversion.

#### Example

Taken from the linked article - it may be tempting to implement code as follows:

```cpp
class Rational
{
public:
    Rational(int numerator = 0,
             int denominator = 1);
    int numerator() const;
    int denominator() const;

    const Rational operator*(const Rational& rhs) const;
private:
    int _numerator;
    int _denominator;
};
```

In most cases, this code will work as expected:

```cpp
Rational one_eighth(1,8);
Rational two(2);
Rational result = one_eighth * two;
result = one_eighth * 2; // this works because 2 gets implicitly converted to a Rational
```

However, it doesn't always:

```cpp
result = 3 * one_eighth; // this one breaks!
```

This is because the literal `3` would need to be _explicitly_ converted to a `Rational` before the `operator*` can be applied, since the operator is defined _inside_ the `Rational` class (thus specifying the LHS type). Moving the operator definition out to be a non-member function fixes this behaviour, as now operands on _both_ sides of the operator can be implicitly converted:

```cpp
class Rational
{
public:
    Rational(int numerator = 0,
             int denominator = 1);
    int numerator() const;
    int denominator() const;

private:
    int _numerator;
    int _denominator;
};
// defining this function outside of the class allows implicit conversion of *both* types
const Rational operator*(const Rational& lhs,
                         const Rational& rhs)
{
    return Rational(lhs.numerator() * rhs.numerator(),
                    lhs.denominator() * rhs.denominator());
}
```

### Use initialiser lists!

Wherever possible, initialise a class's data using initialiser lists instead of in the constructor body. This has several benefits, such as allowing the initialisation of `const`s with values provided in the constructor.

#### Correct

```cpp
class MyClass
{
public:
    MyClass(uint32_t number1, uint32_t number2)
        : _number1(number1),
          _number2(number2) {}

private:
    uint32_t _number1;
    uint32_t _number2;
};
```

#### Incorrect

```cpp
class MyClass
{
public:
    MyClass(uint32_t number1, uint32_t number2)
    {
        _number1 = number1;
        _number2 = number2;
    }

private:
    uint32_t _number1;
    uint32_t _number2;
};
```

### Consider making virtual functions non-public, and public functions non-virtual

If a method is virtual, it is specifying an _implementation detail_. However, if it is public, it is also specifying the _interface_ with which to use the class - these two things are conflicting.

### Destructors should (probably) be virtual

See [Virtuality](http://www.gotw.ca/publications/mill18.htm), specifically that "A base class destructor should be either public and virtual, or protected and nonvirtual." This means that derived classes can be deleted using pointers to base classes. Ideally you can avoid the need to write a destructor at all, but if you can't, then be sure that you've fully considered the ramifications that doing so brings.

### Consider support for a non-throwing swap

If a `swap` function can throw, then it defeats the purpose - if the function throws an exception mid-execution, what state are the inputs left in?

### Avoid providing implicit conversions

Although [implicit conversions](https://en.cppreference.com/w/cpp/language/implicit_conversion) are convenient, they can also cause insidious hard-to-debug issues when they are called unexpectedly. Implicit conversions have their uses, just think twice before implementing them - will you be potentially introducing bugs later down the line? Would making the conversion `explicit` reduce the opportunities for those bugs?

## General Code Guidelines

### Prefer pass-by-reference-to-const to pass-by-value

If this is not done, it means that values can get _copied_ around, which can have performance hits, especially if the objects being copied are complex classes that need to run copy constructors and/or destructors to do so.

#### Acceptable

```cpp
int process_class(const ExampleClass& input_data);
```

#### Incorrect

```cpp
int process_class(ExampleClass input_data);
```

### Memory Management

Never use C `malloc()` and `free()` - there are better C++ replacements.
Firstly, prefer to use references if you can (`type_t& variable`), eliminating pointers entirely.
If a pointer doesn't serve your needs, the next best options (C++ smart pointers) are {expr}`std::unique_ptr` for a pointer which is owned in a single place, and {expr}`std::shared_ptr` for a pointer which needs to be, well, shared between multiple pieces of code (you can research {expr}`std::weak_ptr` in your own time - it's unlikely to be useful).
Smart pointers should be initialised with {expr}`std::make_unique` or {expr}`std::make_shared` as appropriate - they are drop-in, safer replacements for `new`.
Speaking of, avoid the use of `new` and `new[]`.
If you _must_ use it, assign the resultant pointer to a smart pointer type as soon as possible.
Dynamic array allocation can also be almost entirely replaced with C++ STL storage containers such as {expr}`std::vector` (especially with functions like {expr}`std::vector::emplace_back`, which will construct the variable appended to the vector).

Finally, you should be avoiding manual memory management at all costs if you can.
It's just very hard to get right, and if it goes wrong, it can cause a whole host of new and interesting bugs to appear.
If you can use normal initialisation instead, do so.

### Use the STL

The STL (Standard Template Library) is _massive_ in modern C++ and contains a huge number of solutions to common problems - the most common likely being {expr}`std::string` and {expr}`std::vector`.
Less well known solutions are libraries such as `<atomic>`, `<chrono>`, and `<tuple>` - all of which solve common problems well.
Before implementing a new solution, take half a minute to do a search and see if the STL has a solution already.

### When to use `auto`

The `auto` keyword should only be used in cases where it is explicitly clear what the type is. It is very easy to accidentally perform an implicit cast with `auto` to a type you didn't intend to use, and as such should be avoided for standard primitives. There are also cases where explicitly specifying the type may result in clearer code - in these cases don't use it.

Using the STL can result in... interesting... types sometimes - if you don't particularly care what type the variable _is_ (such as `std::vector::begin`), you may use the `auto` (or `const auto`, or `auto&`, or `const auto&` as appropriate) keyword instead of specifying the complete type - **but only do this where you know exactly what will happen!** It should only be used in cases where the type is fully specified and the intended use is clear.

### Use of exceptions

Exceptions are, as the name suggests, for _exceptional_ behaviour. Before writing code which throws exceptions _or_ returns an error code, read through, at the very least, the first 8 items of the [ISO C++](https://isocpp.org/wiki/faq/exceptions) exceptions and error handling page - everything in that document is applicable and should be followed, but the first entries are the most important. The most important points are that exceptions separate the _happy path_ (everything succeeded) from the _bad path_ (errors occurred). Exceptions should not be used as another way to return ordinary data from a function - they should be reserved for errors only. Additionally, they should not be used for flow control - this is what if/else statements are for! Your code should both catch "expected" exceptions, and throw exceptions if it encounters _unexpected_ states. Errors which are part of normal operation, however, should perhaps be handled in other ways (eg `std::optional`). Good reasons to throw exceptions are:

- An error occurs inside a class constructor (**this is what makes RAII possible**)
- A syscall fails (eg `open()` fails)

_Potentially_ valid reasons to throw exceptions, but which require more consideration, are:

- The function received invalid data (depending on the function and what it's meant to do, this could be a valid or invalid reason)

_Bad_ reasons to throw exceptions are:

- An expected and recoverable error occurred
- Internal state is corrupted or assumptions are violated (violations of invariants) - this is what `assert` is for!
- You want to return a different data type from your function - use `std::variant` or redesign your code.

Exceptions should all be derived from the `std::exception` base class. No exceptions (pun intended). You should also **never ever** throw an exception from a class _destructor_ since this causes a whole bunch of nasty behaviour - there's no good way to handle this happening. Finally, try to make `try`/`catch` blocks as short as is reasonably possible.

### Error Return and Retrieval

Error codes should be provided by an `enum` (or `enum class` if appropriate), not integers. On this, do not use `#define`s to specify error codes.

Additionally, do not use return codes to specify a function's success/failure. Either use exceptions to signal that the function failed entirely, or use an appropriate STL type such as `std::optional` to make the function's operation and usage clear. This both makes intentions clearer _and_ forces programmers to think about error handling explicitly, rather than simply ignoring a return value. However, unless the function's behaviour is documented, this is useless! Make sure you document all possible return values or exceptions clearly.

### Avoid `#define` statements (mostly)

While `#define` statements are useful, most of the time it would be better to replace them with an appropriate enum or set of constants inside a class. Additionally, defining macros for simple operations (such as calculating a voltage divider) may also be useful. However, no matter whether it's an ordinary define statement or a macro, keep in mind that it's effectively running a find and replace at compile time, which may have unintended consequences in certain situations.

### Use `assert`s liberally

It is better to have an `assert` clearly specifying assumptions (and throwing errors when things go wrong) than to have later bugs because those assumptions were violated. Just make sure that they don't have side effects (such as variable assignments) - compiling out `assert`s should not change the code's functionality.

### Use C++ casts

The C-style `(type)var` casts can have a vast number of unintended side effects. C++ provides a set of casts which specify your intentions much more explicitly and can be safer to use as well, such as:

```cpp
int number = 0;
unsigned int unsigned_number = static_cast<unsigned int>(number);
```

The `static_cast` is almost certainly the most appropriate for your needs (followed by `dynamic_cast` in the majority of other cases), but if it isn't, check which one would be best suited to your needs - [this](https://cplusplus.com/doc/tutorial/Typecasting/) article provides a good explanation of the different types of casting and their side effects.

## Types

### Use `typedef`

If using `typedef` to alias types in the STL, functions, or complex types makes things simpler or easier to understand, do so.

### Use meaningful types

Types can convey important information about your intentions. For example:

- `size_t` conveys that it's an unsigned integer size/length of something
- `std::chrono::duration` is explicitly a duration between two points
- `std::optional` is explicitly a value which may or may not exist
  These are just a few examples, but in short, consider whether the type you're using conveys the information that it should, and if it doesn't, check to see if there isn't a better suited type.

### Integer types

Use the standard integer types in `<cstdint>` and `<climit>`. Prefer to explicitly state the size of the type - that is, prefer `int64_t` over `long`, or `uint8_t` over `unsigned char`.

- Use `size_t` for unsigned integer types denoting length, size, or capacity.
- Use `ssize_t` when you need a _signed_ integer type for the same reasons, but where a negative value may be returned on an error (do not _write_ code that does this! See above about error handling, only use this when libraries return it).
- Use `off_t` for file positions and offsets.
- Use `ptrdiff_t` for integers representing offset or difference between pointers.
