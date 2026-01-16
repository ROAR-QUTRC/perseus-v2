# JS/TS Standards

These standards are based on the [MDN Developer Standards](https://developer.mozilla.org/en-US/docs/MDN/Writing_guidelines/Code_style_guide/JavaScript) and known ways of avoiding the famous JavaScript weirdness.

:::{note}
When reading these standards "primitives" or "primitive types" refers to anything that isn't an object and is not a function:

- string
- number
- bigint
- undefined
- null
- symbol

Note: if you are unsure what symbols are that's fine as they are primarily useful for controlling what data is exposed by libraries.
:::

## Naming

For any JavaScript, TypeScript or Svelte code there are a number of naming conventions that contradict other language standards for perseus-v2:

- Variables, functions and files should use **camelCase**
- Svelte components when referenced in markdown should use **PascalCase**
- CSS classes should be written in **kebab-case**

## Type Safe Comparisons

Most languages use `==` for comparison. While this syntax is supported by JS it is not type safe and may return unexpected results. For example:

```js
"8" == 8; // true
false == "0"; // true
0 == []; // true
```

Only `===` Should be used and values should be cast before comparison if they may be of different types.

```js
let foo: number = 3
let bar: string | number = "3"

foo === Number(bar) // Correct
foo == bar // Incorrect
```

## Null and Undefined

To be clear, null means intentionally no value, and undefined means a property does not exist or a variable is uninitialised.
You should only use undefined when removing a property from an object otherwise you should be using null.

## Handling Nullish Values

Uses of the optional modifier to suppress warnings should be in conjunction with the nullish coalescing operator (`??`) to provide a fall back value where possible.

```js
const foo: number[] | undefined = []

// let length = foo.length
let length = foo?.length ?? 0
```

The non-null operator (`!`) should be used sparingly and if it has been chained multiple times this is an indicator that you should change your type definitions.
When checking if a value is truthy or falsy, always prefer the boolean evaluation shorthand over explicit checks unless you are targeting a falsy value.

```js
let foo?: string | null = ""

if (foo) {
    // Truthy value
} else if (foo === "") {
    // Falsy but specifically ""
}
```

## Spreading

To avoid unexpected overrides while spreading, the spread value should appear as the first in the comma separated list.

```js
let draft = {
  name: "ted",
  nickname: "teddy",
  age: 19,
};
let name = "bruce";

let user = { ...draft, name }; // { name: "bruce", nickname: "teddy", age: 19 }
```

## Constants

In JS/TS `const` should be used instead of `let` for any variable that will never be reassigned.
If you are unsure check if either of these criteria are met:

- Variables of a primitive type that should never change
- Arrow functions, objects, and arrays that can be mutable but are never nullish
  If an object or some of its members should be immutable then use the `as const` type cast at the end of the object/array where it is initialised. Alternatively, use the `ReadOnly<Type>` utility function to specify the same thing in the type definition.

## Types, Interfaces and Classes

All three of these are ways to define JS objects with different levels of control over the structure of the object.

- Type - This should be used to mutate and combine existing types: `type driver = Pick<User, "name" | "age"> & Licence // Union of License and select properties of User`
- Interface - This is a minimal object template that includes no runnable code and only Type information
- Class - This is an extension of the interface that allows for default values and constructors to be set

## Loops

Built in shorthands and functions should be used where possible instead of c style for Loops. For strings and arrays the `forEach` function or a `for ... of` loop should be used. While loops are typically unnecessary and should be avoided as they are blocking and can quickly cause the browser to stop responding. Lengthy operations should always be done serverside and then the results should be passed back to the client asynchronously.

## Always Use Semicolons

Semicolons should be used primarily to improve code readability and avoid strange syntax like:

```js
foo = bar;
(await myPromise).memberFunction();
```

Without the semicolon this would append the second line to the variable `bar` and try and call it.

## Always Use Object Shorthands

This reduces the amount of code, improves readability and helps name variables.

```js
let name = "Jim";
let age = 13;

// Both foo and bar are identical except bar is easier to read
const foo = {
  name: name,
  age: age,
};

const bar = {
  name,
  age,
};
```

Svelte has a very similar shorthand for dynamic element [props](https://svelte.dev/docs/svelte/$props)

## Avoid Strings Near Operators

When an arithmetic operator is used next to a string, the string is implicitly cast to a number causing potentially unexpected behaviour.
Additionally the + operator is supported with strings to concatenate them. This should not be used in favour of template literals to preserve spacing and improve readability.

## Literals

When initialising variables always use literals instead of their constructors.
However type definitions should use utility functions with generics (`Array<Type>` instead of `[]`).

```js
// Bad
let myString: string = new String() // Do not use constructor
let myArray: string[] = new Array<string>() // Do not use literals in type definitions

// Good
let myString: string = ""
let myArray: Array<string> = []
```

## When to Use Types

When the full type expected can be inferred from the value used to initialise the variable no typing is necessary. All other cases should be completely typed including `unknown` when the type is unknown. The type `any` should only be used as a placeholder.

Not all type errors are worth the effort!
While typescript improves the developer experience significantly, sometimes type errors can be more effort than they are worth when the code has been tested and runs correctly. The annotation @ts-expect-error can be used sparingly.
