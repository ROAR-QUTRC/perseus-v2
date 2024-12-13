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
using namespace addressing;
using namespace addressing::legacy;
using namespace addressing::legacy::drive;
using namespace parameters::legacy::drive;
using namespace addressing::legacy::drive::motors;
using namespace parameters::legacy::drive::motors;
int main()
{
    cout << "Pre-initialisation" << endl;
    auto can = RawCanInterface("vcan0");
    EscParameterGroup groupFL(address_t(
        SYSTEM_ID,
        SUBSYSTEM_ID,
        static_cast<uint8_t>(device::FRONT_LEFT_MOTOR)));
    EscParameterGroup groupFR(address_t(
        SYSTEM_ID,
        SUBSYSTEM_ID,
        static_cast<uint8_t>(device::FRONT_RIGHT_MOTOR)));
    EscParameterGroup groupRL(address_t(
        SYSTEM_ID,
        SUBSYSTEM_ID,
        static_cast<uint8_t>(device::REAR_LEFT_MOTOR)));
    EscParameterGroup groupRR(address_t(
        SYSTEM_ID,
        SUBSYSTEM_ID,
        static_cast<uint8_t>(device::REAR_RIGHT_MOTOR)));

    PacketManager packetManager(can);
    packetManager.addGroup(groupFL);
    packetManager.addGroup(groupFR);
    packetManager.addGroup(groupRL);
    packetManager.addGroup(groupRR);

    // can.addFilter(filter_t{
    //     .address = flagged_address_t(),
    //     .mask = 0,
    // });

    for (int i = 0; i < 200; i++)
    {
        packetManager.handle();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto& speed = groupFL.getSpeed();
    speed.enabled = true;
    speed.direction = motor_direction::FORWARD;
    speed.speed = 0x77AA;
    for (int i = 0; i < 200; i++)
    {
        packetManager.handle();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto& status = groupFL.getStatus();
    can.receive(true);
    cout << "stat " << status.ready << " " << status.realSpeed << " " << status.realCurrent << endl;
    cout << "spd " << speed.enabled << " " << (int)speed.direction << " " << speed.speed << endl;
    speed.direction = motor_direction::REVERSE;
    for (int i = 0; i < 200; i++)
    {
        packetManager.handle();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // can_data d;
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
    // if (const auto packet = can.receive(true); packet.has_value())
    // {
    //     cout << "Received a packet" << endl;
    //     cout << "Data: ";
    //     auto data = packet->getData();
    //     for (size_t i = 0; i < packet->getDataLen(); i++)
    //         cout << (int)data[i] << " ";
    //     cout << endl;
    // }
}