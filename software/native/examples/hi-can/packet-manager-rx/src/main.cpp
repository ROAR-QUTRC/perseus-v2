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
void rx_callback_1(const Packet&);
void rx_callback_2(const Packet&);
void print_frame_data(const Packet&);
void timeout_callback();
void recovery_callback(const Packet&);

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

        addressing::filter_t filter_1{
            .address = 0x12345678,
            .mask = 0xFFFF0000,
        };
        PacketManager::callback_config_t config_1{
            .data_callback = rx_callback_1,
        };
        packet_manager.set_callback(filter_1, config_1);

        addressing::filter_t filter_2{
            .address = 0x10005678,
            .mask = 0xFFFF0000,
        };
        PacketManager::callback_config_t config_2{
            .data_callback = rx_callback_2,
            .timeout_callback = timeout_callback,
            .timeout_recovery_callback = recovery_callback,
            .timeout = 1s,
        };
        packet_manager.set_callback(filter_2, config_2);

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

void rx_callback_1(const Packet& frame)
{
    cout << "From RX Callback 1!" << endl;
    print_frame_data(frame);
}

void rx_callback_2(const Packet& frame)
{
    cout << "From RX Callback 2!" << endl;
    print_frame_data(frame);
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

void timeout_callback()
{
    cout << "Timeout callback called!" << endl;
}

void recovery_callback(const Packet& frame)
{
    (void)frame;  // silence unused warning
    cout << endl
         << "Recovery callback called!" << endl;
}