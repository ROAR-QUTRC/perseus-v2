// leader-main-mini.cpp
#include <iostream>
#include <thread>

#include "arm-network-mini.hpp"
#include "perseus-arm-teleop-mini.hpp"

int main()
{
    try
    {
        ST3215ServoReaderMini reader("/dev/ttyUSB0", 115200);
        perseus::ArmNetworkInterfaceMini network(perseus::ArmNetworkInterfaceMini::Mode::Client, "127.0.0.1");
        network.start();

        while (true)
        {
            perseus::ServoPositionsMessage msg;
            msg.header = {perseus::PROTOCOL_VERSION, perseus::MessageType::SERVO_POSITIONS, 12};
            for (int i = 0; i < 6; ++i)
            {
                msg.positions[i] = reader.readPosition(i + 1);
            }
            network.sendServoPositions(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}