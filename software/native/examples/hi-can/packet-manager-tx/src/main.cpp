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
std::vector<uint8_t> data_generator();

static bool running = true;

int main(int argc, const char** argv)
{
    std::string interface_id = "vcan0";
    if (argc > 1)
        interface_id = argv[1];

    // handle CTRL+C cleanly: https://stackoverflow.com/questions/1641182/how-can-i-catch-a-ctrl-c-event
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

        addressing::flagged_address_t address(0x12345678);
        PacketManager::transmission_config_t config = {
            .generator = data_generator,
            .interval = 1s,
        };
        packet_manager.set_transmission_config(address, config);

        while (running)
        {
            packet_manager.handle();
            std::this_thread::sleep_for(10ms);
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
    (void)signal;  // silence unused variable warning
    cout << endl
         << "SIGINT caught, shutting down..." << endl;
    running = false;
}

std::vector<uint8_t> data_generator()
{
    static uint8_t counter = 0;
    counter++;
    cout << "Generating data, counter: " << counter << endl;
    const std::vector<uint8_t> data = {counter, 0x11, 0x22, 0x33};
    return data;
}