#include <hi_can_raw.hpp>
#include <iostream>
#include <string>

using namespace hi_can;
using std::cout, std::cin, std::endl;

void print_frame_data(const Packet&);

int main(int argc, const char** argv)
{
    std::string interface_id = "vcan0";
    if (argc > 1)
        interface_id = argv[1];

    try
    {
        RawCanInterface can_interface(interface_id);
        cout << "Opened CAN interface: " << interface_id << endl;

        can_interface.set_receive_callback(print_frame_data);
        cout << "Waiting to receive frame..." << endl;
        can_interface.receive_all(true);
    }
    catch (const std::exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
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
