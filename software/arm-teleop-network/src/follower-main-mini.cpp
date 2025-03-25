// follower-main-mini.cpp
#include <iostream>
#include <thread>

#include "arm-network-mini.hpp"
#include "perseus-arm-teleop-mini.hpp"

int main()
{
    try
    {
        ST3215ServoReaderMini reader("/dev/ttyUSB0", 115200);
        perseus::ArmNetworkInterfaceMini network(perseus::ArmNetworkInterfaceMini::Mode::Server);
        network.start();

        network.setServoCallback([&reader](const perseus::ServoPositionsMessage& msg)
                                 {
            for (int i = 0; i < 6; ++i) {
                reader.writePosition(i + 1, msg.positions[i]);
            } });

        while (true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}