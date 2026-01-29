# Using the `PacketManager`

```{namespace} hi_can

```

In this tutorial, you will learn how to use the Hi-CAN {expr}`PacketManager` class to handle managing parameter receptions and scheduled transmissions.

## Requirements

Before starting this tutorial, follow the [](../hi-can-index.md#setup) here.

## Receive Callbacks

````{note}
The program written in this section of the tutorial can be run with:

```console
nix run .#pkgs.examples.hi-can.packet-manager-rx
```

````

As before, we will be starting off by creating a {expr}`RawCanInterface` instance to interact with the bus, and a function to print out frame data, which in this case is just the renamed {cpp-inline}`rx_callback()` function from the previous tutorial.
However, we will also be instantiating a {expr}`PacketManager` and calling {expr}`PacketManager::handle()` in a main loop.
Since this code now includes an infinite main loop, we also need to add a `SIGINT` handler so we can use {keys}`ctrl+c` to shut the program down cleanly.

```{code-block}
:linenos:
:emphasize-lines: 1-4, 8, 12, 15, 24-29, 36-43, 54-58
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <hi_can_raw.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace hi_can;
using namespace std::chrono_literals;
using std::cout, std::cin, std::endl;

void signal_handler(int signal);
void print_frame_data(const Packet&);

int main(int argc, const char** argv)
{
    std::string interface_id = "vcan0";
    if (argc > 1)
        interface_id = argv[1];

    // handle CTRL+C cleanly
    struct sigaction sigint_handler;
    sigint_handler.sa_handler = signal_handler;
    sigemptyset(&sigint_handler.sa_mask);
    sigint_handler.sa_flags = 0;
    sigaction(SIGINT, &sigint_handler, NULL);

    try
    {
        RawCanInterface can_interface(interface_id);
        cout << "Opened CAN interface: " << interface_id << endl;

        PacketManager packet_manager(can_interface);

        bool running = true;
        while (running)
        {
            packet_manager.handle();
            std::this_thread::sleep_for(10ms); // (1)
        }
    }
    catch (const std::exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}

void signal_handler(int signal)
{
    cout << "SIGINT caught, shutting down..." << endl;
    running = false;
}

void print_frame_data(const Packet& frame)
{
    // pull out the data we want
    const auto& address = frame.get_address();
    const auto& data = frame.get_data();

    // print out the address info
    cout << std::format("Address: {:#10x}\tExtended: {}\tRTR: {}\tError: {}\t",
                        address.address,
                        address.is_extended,
                        address.is_rtr,
                        address.is_error)
         << endl;

    // and data
    cout << "Data length: " << data.size() << " bytes" << endl;
    if (data.size() > 0 && !address.is_rtr)
    {
        cout << "Data: ";
        for (const auto byte : data)
            cout << std::format("{:#04x} ", byte);
        cout << endl;
    }
}
```

```{code-annotations}
1. The {cpp-inline}`using namespace std::chrono_literals;` line allows us to define {expr}`std::chrono::steady_clock::duration` intervals as numbers with unit suffixes, like `10ms` or `5s`.
```

```{important}
You may note on line 31 that there is a delay in the main loop.
Since we're calling {expr}`PacketManager::handle()` in non-blocking mode, without a delay the loop will run as fast as it can and max out the CPU core it's running on.
Alternatively, if we _did_ call it in blocking mode, it would idle until a frame is received, but also be unable to run scheduled transmissions except directly after receiving a frame.
```

With the setup complete, the next step is to configure it to actually _do_ something.
We can configure the {expr}`PacketManager` to call specific functions when frame IDs matching a filter are received with {expr}`PacketManager::set_callback()`.
Obviously, to do that, we need a filter:

```{literalinclude} /../../software/native/examples/hi-can/packet-manager-rx/src/main.cpp
:start-at: filter_1
:end-at: };
:prepend: // ...
:append: // ...
```

This particular filter will receive any frames with an ID starting with `0x1234`.
Even though the full address it's set to match is `0x12345678`, since the last 4 digits aren't included in the mask, they will be ignored in the comparison.
The other component we need is a valid callback configuration, then we can configure the {expr}`PacketManager`:

```{literalinclude} /../../software/native/examples/hi-can/packet-manager-rx/src/main.cpp
:start-at: config_1
:end-at: set_callback
:prepend: // ...
:append: // ...
```

This particular configuration will just call the {cpp-inline}`rx_callback_1` function whenever it receives data.
Repeating the setup with another filter gives the full program:

```{literalinclude} /../../software/native/examples/hi-can/packet-manager-rx/src/main.cpp
:linenos:
:emphasize-lines: 16-17, 19-20, 44-63, 88-98, 125-135
```

The second configuration in {cpp-inline}`config_2` also demonstrates the _timeout_ functionality of the {expr}`PacketManager`.
As mentioned in the [specification](project:/systems/can-bus.md#callbacks), after 3 times the duration of whatever {expr}`PacketManager::callback_config_t::timeout` is set to---representing 2 missed frames---{expr}`PacketManager::callback_config_t::timeout_callback` will be called.
Here, that means you should see the timeout message on your terminal 3 seconds after running the program.
If you send a message matching that filter:

```console
cansend vcan0 10000000#11223344
```

You should see, in order:

1. The data callback be called (RX callback 2), which will then print the frame data.
2. The timeout recovery callback get called.
3. 3 seconds later, the timeout callback message.

Note that although both are provided in this tutorial, you only _need_ to provide one of the timeout or recovery callbacks.

However, if you instead send a frame matching only the first configuration:

```console
cansend vcan0 12340000#11223344
```

The RX callback _1_ function will be called instead, since its filter now matches the frame.

That's it!
You now know how to use the {expr}`PacketManager` to handle receiving data and routing it to a callback.

## Transmission Configurations

````{note}
The program written in this section of the tutorial can be run with:

```console
nix run .#pkgs.examples.hi-can.packet-manager-tx
```

````

Much like with the {expr}`RawCanInterface` directly, transmitting data is much simpler than receiving it.
To set up an interval transmission, we need a {expr}`addressing::flagged_address_t` for the transmitted frame's ID and flags, and a valid {expr}`PacketManager::transmission_config_t` for the data and its settings.
However, rather than just setting the data directly, transmissions instead call a _function_ called the "data generator" which must return the frame's data as a {expr}`std::vector<uint8_t>`: {expr}`PacketManager::transmission_config_t::generator`.
Although this might seem strange at first (why not just update the data in the {expr}`PacketManager`?), it means that rather than the _data source_ needing to know about the transmission, the _transmission_ knows about the data source, removing code duplication in every potential data generator.

```{literalinclude} /../../software/native/examples/hi-can/packet-manager-tx/src/main.cpp
:lines: 70-
```

Putting it all together gives:

```{literalinclude} /../../software/native/examples/hi-can/packet-manager-tx/src/main.cpp
:emphasize-lines: 16, 40-45, 70-
```

````{tip}
Run
```console
candump any
```
to see the frames being transmitted!
````

```{namespace-push} PacketManager

```

The remaining config options are the interval determining the time between frames, and the optional {expr}`transmission_config_t::should_transmit_immediately` flag, which if {expr}`true` will force the frame to be transmitted as soon as the configuration is set, rather than waiting till the interval expires for the first time.

```{namespace-pop}

```

Finally, as with the {expr}`RawCanInterface`, the {expr}`PacketManager` also has split receive and transmit functions: {expr}`PacketManager::handle_receive()` and {expr}`PacketManager::handle_transmit()`.
The only difference is that the `handle_transmit()` function takes a flag which, if set, will force _all_ the registered transmission configurations to be sent **immediately**, regardless of configuration.
Also, since {expr}`PacketManager::handle()` calls both of them internally, if the receive function is blocking, it can no longer _transmit_ any data.
