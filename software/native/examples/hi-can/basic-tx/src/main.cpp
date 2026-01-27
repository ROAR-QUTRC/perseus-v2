#include <hi_can_raw.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace hi_can;
using std::cout, std::cin, std::endl;

int main(int argc, const char** argv)
{
    std::string interface_id = "vcan0";
    if (argc > 1)
        interface_id = argv[1];

    try
    {
        RawCanInterface can_interface(interface_id);
        cout << "Opened CAN interface: " << interface_id << endl;

        addressing::standard_address_t txAddress(0x1F, 0, 0, 0, 0);
        const std::vector<uint8_t> txData{1, 2, 3, 4, 5, 6, 7, 8};
        Packet txPacket(addressing::flagged_address_t(txAddress), txData);

        cout << "Transmitting frame with ID "
             << std::format("{:#04x}",
                            static_cast<addressing::raw_address_t>(txAddress))
             << endl;
        can_interface.transmit(txPacket);
    }
    catch (const std::exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}