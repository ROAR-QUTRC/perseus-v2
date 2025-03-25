// leader-main-mini.cpp
#include <iostream>
#include <thread>

#include "arm-network-mini.hpp"
#include "perseus-arm-teleop-mini.hpp"
// In leader-main-mini.cpp
int main()
{
    try
    {
        std::cout << "Starting leader program..." << std::endl;
        ST3215ServoReaderMini reader("/dev/ttyACM0", 115200);
        std::cout << "Serial port opened successfully" << std::endl;

        perseus::ArmNetworkInterfaceMini network(perseus::ArmNetworkInterfaceMini::Mode::Client, "192.168.1.254");
        std::cout << "Network interface created" << std::endl;

        if (network.start())
        {
            std::cout << "Network started successfully" << std::endl;
        }
        else
        {
            std::cout << "Failed to start network" << std::endl;
        }

        while (true)
        {
            perseus::ServoPositionsMessage msg;
            msg.header = {perseus::PROTOCOL_VERSION, perseus::MessageType::SERVO_POSITIONS, 12};
            try
            {
                for (int i = 0; i < 6; ++i)
                {
                    msg.positions[i] = reader.readPosition(i + 1);
                    std::cout << "Servo " << (i + 1) << " position: " << msg.positions[i] << std::endl;
                }
                network.sendServoPositions(msg);
                std::cout << "Positions sent. Connection status: "
                          << (network.isConnected() ? "Connected" : "Disconnected") << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error reading positions: " << e.what() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Slower update for debug
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}