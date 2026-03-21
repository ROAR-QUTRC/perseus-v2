# Basic Hi-CAN Usage

```{namespace} hi_can

```

In this tutorial, you will learn the basics of transmitting and receiving data with Hi-CAN.

## Requirements

Before starting this tutorial, follow the [](../hi-can-index.md#setup) here.

## Receiving Data

````{note}
The program written in this section of the tutorial can be run with:

```console
nix run .#pkgs.examples.hi-can.basic-rx
```

````

If you've read through the main CAN [document](project:/systems/can-bus.md), you'll know that we need a {expr}`CanInterface` instance to work with the bus and receive a {term}`CAN frame`, represented with a {expr}`Packet` containing the actual data.
Since these examples are all intended to run on Linux, we need to use the `hi-can-raw` implementation to get our CAN interface:

```cpp
#include <hi_can_raw.hpp>
#include <iostream>
#include <string>

using namespace hi_can;
using std::cout, std::cin, std::endl;

int main()
{
    std::string interface_id = "vcan0";
    RawCanInterface can_interface(interface_id);
    cout << "Opened CAN interface: " << interface_id << endl;

    return 0;
}
```

If the bus in the `interface_id` exists, great!
You just created your first CAN interface, and the program exited after printing out the message.
However, if it doesn't, you might get an error that looks something like this:

```none
terminate called after throwing an instance of 'std::runtime_error'
  what():  Failed to get interface index for "can0": No such device
[1]    39092 abort      ./hi_can_basic_error
```

This is because our program currently has no error handling, and therefore crashed trying to connect to the bus.
Following the [software standards](project:/standards/software-index.md), `hi-can` uses exceptions for (unusual) error handling!
Either the calls succeed, they are expected to fail and the return type reflects that, or they fail and throw an exception.
Therefore, we need to wrap code that can potentially fail in a `try`/`catch` block:

```{code-block}
:emphasize-lines: 12-13, 16-21

#include <hi_can_raw.hpp>
#include <iostream>
#include <string>

using namespace hi_can;
using std::cout, std::cin, std::endl;

int main()
{
    std::string interface_id = "vcan0";

    try
    {
        RawCanInterface can_interface(interface_id);
        cout << "Opened CAN interface: " << interface_id << endl;
    }
    catch (const std::exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}
```

This time, the error is caught before it can crash the program, and it cleanly prints out the error message:

```none
Error: Failed to get interface index for "can0": No such device
```

To make our lives easier, let's also make the program use the CAN bus name we give it as an argument, if provided:

```{literalinclude} /../../software/native/examples/hi-can/basic-rx/src/main.cpp
:start-at: int main
:end-at: interface_id = argv
:emphasize-lines: 1, 4-5
:append: // ...

```

However, just having a connection to the bus is no use if we don't use it!

````{error}
If you're getting an that looks like:
```none
Error: Failed to create CAN socket: Address family not supported by protocol
```
This usually means the CAN bus you're trying to open does not exist.
In these tutorials, that probably means you're trying to use `vcan0` but haven't run the setup script as described [here](../hi-can-index.md#setup).
````

Now we need to actually _receive_ data.
First, a function to process received frames:

```{literalinclude} /../../software/native/examples/hi-can/basic-rx/src/main.cpp
:start-at: void rx_callback(const Packet& frame)

```

Which, in this case, just prints out all the information about the frame.
Next, to actually receive data.

The {expr}`CanInterface::receive()` function has two modes, _blocking_, and _non-blocking_.
In blocking mode, it will wait indefinitely for a frame to arrive, and immediately return a frame once it's received.
If a frame has _already_ arrived and is buffered internally, it won't block at all, since it can just return that frame.
Alternatively, in non-blocking mode, it will return a buffered frame if possible, or {expr}`std::nullopt` otherwise.
In this case, we will use it in blocking mode, so we can guarantee that it returns a frame.

```cpp
// ...
void rx_callback(const Packet&);

int main(int argc, const char** argv)
{
    // ...
        cout << "Waiting to receive frame..." << endl;
        const auto frame = can_interface.receive(true);
        rx_callback(*frame);
    // ...
}
```

```{tip}
To send data on the bus, use {manpage}`cansend(1)`.
For example, on `vcan0`: `cansend vcan0 12345678#1122334455667788`
```

This is the most basic way to receive data from the bus.
However, it is also the worst.
The `receive()` function only handles a single frame at a time, so if you want to receive more than one you need to call it multiple times.
Additionally, you need to deal with error handling if you're using it in non-blocking mode and it _doesn't_ receive a frame.
The preferred way to receive data is instead to set the interface's _receive callback_.
This is a function that gets called every time the interface processes an incoming frame:

```{literalinclude} /../../software/native/examples/hi-can/basic-rx/src/main.cpp
:start-at: can_interface.set_receive
:end-at: can_interface.receive
:prepend: // ...
:append: // ...

```

Using this then allows us to use the {expr}`CanInterface::receive_all()` function instead of `receive()`.
`receive_all()` can't return frames, so you **must** set a receive callback to use it, but it is much more convenient for actual use, since if there are multiple buffered frames, it will process all of them at once.

Putting it all together:

```{literalinclude} /../../software/native/examples/hi-can/basic-rx/src/main.cpp

```

This program will open `vcan0` by default, or any bus you specify, wait to receive a CAN frame, print out the frame, and then exit.

````{tip}
To receive from _any_ bus, use `any` instead of a bus name:

```console
nix run .#pkgs.examples.hi-can.basic-rx -- any
```

If you do this, you will not be able to _transmit_ frames, however.
````

## Transmitting Data

````{note}
The program written in this section of the tutorial can be run with:

```console
nix run .#pkgs.examples.hi-can.basic-tx
```

````

Like before, we will need a valid {expr}`CanInterface` instance to transmit data, but this time we will also need to create a {expr}`Packet` containing the data to actually transmit.
However, setting up the interface is exactly the same as before:

```{literalinclude} /../../software/native/examples/hi-can/basic-tx/src/main.cpp
:emphasize-lines: 20-28

```

To build a `Packet`, we first need an address for it.
In this case, we're using the {expr}`addressing::standard_address_t`, which constructs the {term}`frame ID` from system, subsystem, device, group, and parameter IDs.
Next is data---although it is optional.
Since most of the time data needs to be {term}`serialised <serialisation>`, returning a {expr}`std::vector<uint8_t>` data buffer, we will directly create a buffer too.
However, the `Packet` needs a `addressing::flagged_address_t`, not just an ordinary address.
Fortunately, it can be directly constructed from the standard address.

```{tip}
To view CAN bus data, use {manpage}`candump(1)`.
You probably want to receive from any bus, so run: `candump any`.
```

That's all there is to it!
Actually sending data is fairly simple---the trick is implementing the whole Hi-CAN standard, particularly address construction and interval transmission.
Fortunately, that's what the {expr}`PacketManager` class is for, and the next tutorial teaches you how to use it!

## Completed Code

All source code examples can be found under {file}`/software/native/examples`.
