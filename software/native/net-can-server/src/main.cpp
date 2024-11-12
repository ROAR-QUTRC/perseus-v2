#include <hi_can_raw.hpp>
#include <iostream>
#include <thread>

#pragma pack(push, 1)
struct can_data
{
    uint8_t data[4];
};
#pragma pack(pop)

using namespace hi_can;
using namespace std;
int main()
{
    cout << "Pre-initialisation" << endl;
    auto can = RawCanInterface("any");
    can_data d;
    // for (int j = 0; j < 1000; j++)
    // {
    //     for (size_t i = 0; i < sizeof(d.data); i++)
    //     {
    //         d.data[i] = i + j;
    //     }
    //     try
    //     {
    //         can.transmit(Packet(0x1F000000, d));
    //     }
    //     catch (const std::exception& e)
    //     {
    //         cerr << e.what() << '\n';
    //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     }
    // }
    if (const auto packet = can.receive(true); packet.has_value())
    {
        cout << "Received a packet" << endl;
        cout << "Data: ";
        auto data = packet->getData();
        for (size_t i = 0; i < packet->getDataLen(); i++)
            cout << (int)data[i] << " ";
        cout << endl;
    }
}