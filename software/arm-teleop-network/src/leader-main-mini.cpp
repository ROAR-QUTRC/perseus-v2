// leader-main-mini.cpp - robust version
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "arm-network-mini.hpp"
#include "perseus-arm-teleop-mini.hpp"

// Global variables for clean shutdown
static bool running = true;

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << "Received signal " << signum << ", shutting down..." << std::endl;
    running = false;
}

int main(int argc, char* argv[])
{
    // Set up signal handling
    signal(SIGINT, signalHandler);

    // Default follower IP address
    std::string follower_ip = "192.168.1.15";  // Default to assigned static ip address

    // Parse command line arguments
    if (argc > 1)
    {
        follower_ip = argv[1];
    }

    std::string serial_port = "/dev/ttyACM0";
    if (argc > 2)
    {
        serial_port = argv[2];
    }

    try
    {
        std::cout << "Starting leader program..." << std::endl;
        std::cout << "Using serial port: " << serial_port << std::endl;
        std::cout << "Target follower IP: " << follower_ip << std::endl;

        // Initialize servo reader
        ST3215ServoReaderMini reader(serial_port, 115200);  // 115200 baud rate as confirmed
        std::cout << "Serial port opened successfully" << std::endl;

        // Test servo communication
        std::cout << "Testing servo communication..." << std::endl;
        try
        {
            for (int i = 1; i <= 6; i++)
            {
                uint16_t pos = reader.readPosition(i);
                std::cout << "Servo " << i << " current position: " << pos << std::endl;
            }
            std::cout << "Servo communication successful" << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout << "Warning: Servo test failed: " << e.what() << std::endl;
            std::cout << "Will continue anyway, but servo reading may not work" << std::endl;
        }

        // Main loop variables
        int connectionCheckCounter = 0;
        int failedReadCounter = 0;
        bool networkStarted = false;
        std::unique_ptr<perseus::ArmNetworkInterfaceMini> network;

        std::cout << "Entering main loop. Press Ctrl+C to exit." << std::endl;
        std::cout << "Will attempt to connect to follower at " << follower_ip << ":" << perseus::DEFAULT_PORT << std::endl;

        auto lastConnectionAttempt = std::chrono::steady_clock::now();

        // Main loop
        while (running)
        {
            // Try to create network connection if not already established
            if (!networkStarted || (network && !network->isConnected()))
            {
                auto now = std::chrono::steady_clock::now();

                // Only try to reconnect every 5 seconds to avoid excessive CPU usage
                if (!networkStarted ||
                    std::chrono::duration_cast<std::chrono::seconds>(now - lastConnectionAttempt).count() >= 5)
                {
                    lastConnectionAttempt = now;

                    try
                    {
                        // Create fresh network interface each time
                        std::cout << "Creating network interface for " << follower_ip << ":" << perseus::DEFAULT_PORT << std::endl;
                        network = std::make_unique<perseus::ArmNetworkInterfaceMini>(
                            perseus::ArmNetworkInterfaceMini::Mode::Client, follower_ip);

                        std::cout << "Starting network connection..." << std::endl;
                        if (network->start())
                        {
                            std::cout << "Network interface started, attempting connection..." << std::endl;
                            networkStarted = true;
                        }
                        else
                        {
                            std::cout << "Failed to start network interface, will retry in 5 seconds..." << std::endl;
                            networkStarted = false;
                        }
                    }
                    catch (const std::exception& e)
                    {
                        std::cout << "Error creating network interface: " << e.what() << std::endl;
                        std::cout << "Will retry in 5 seconds..." << std::endl;
                        networkStarted = false;
                        network.reset();  // Clean up failed network object
                    }
                }
            }

            // Periodically check connection status
            if (connectionCheckCounter++ % 20 == 0)
            {  // Every ~2 seconds
                if (network)
                {
                    std::cout << "Network status: " << (network->isConnected() ? "CONNECTED" : "ATTEMPTING CONNECTION...") << std::endl;
                }
                else
                {
                    std::cout << "Network status: NOT INITIALIZED (will retry)" << std::endl;
                }
            }

            // Only try to send data if connected
            if (network && network->isConnected())
            {
                perseus::ServoPositionsMessage msg;
                msg.header = {perseus::PROTOCOL_VERSION, perseus::MessageType::SERVO_POSITIONS, 12};

                try
                {
                    // Read positions from all 6 servos
                    for (int i = 0; i < 6; ++i)
                    {
                        msg.positions[i] = reader.readPosition(i + 1);
                    }

                    // Only print positions every ~2 seconds to avoid console spam
                    if (connectionCheckCounter % 20 == 0)
                    {
                        std::cout << "Current servo positions: ";
                        for (int i = 0; i < 6; ++i)
                        {
                            std::cout << msg.positions[i] << " ";
                        }
                        std::cout << std::endl;
                    }

                    // Send the positions to the follower
                    network->sendServoPositions(msg);
                    failedReadCounter = 0;  // Reset failure counter on success
                }
                catch (const std::exception& e)
                {
                    failedReadCounter++;
                    if (failedReadCounter % 10 == 1)
                    {  // Only show occasional errors to avoid spam
                        std::cerr << "Error reading/sending positions: " << e.what() << std::endl;
                    }
                }
            }

            // Sleep to avoid excessive CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Shutting down leader..." << std::endl;
        if (network)
        {
            network->stop();
        }
        std::cout << "Leader shutdown complete" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}