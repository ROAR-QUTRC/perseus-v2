// follower-main-mini.cpp
#include <csignal>
#include <iostream>
#include <thread>

#include "arm-network-mini.hpp"
#include "perseus-arm-teleop-mini.hpp"

// Global variables for clean shutdown
static bool running = true;
static ST3215ServoReaderMini* reader_ptr = nullptr;

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << "Received signal " << signum << ", shutting down..." << std::endl;
    running = false;
}

int main()
{
    // Set up signal handling
    signal(SIGINT, signalHandler);

    try
    {
        std::cout << "Starting follower program..." << std::endl;

        // Open the serial port
        std::cout << "Opening serial port /dev/ttyUSB0..." << std::endl;
        ST3215ServoReaderMini reader("/dev/ttyUSB0", 115200);
        reader_ptr = &reader;
        std::cout << "Serial port opened successfully" << std::endl;

        // Test servo communication
        std::cout << "Testing servo communication..." << std::endl;
        try
        {
            for (int i = 1; i <= 6; i++)
            {
                uint16_t pos = reader.readPosition(i);
                std::cout << "Servo " << i << " current position: " << pos << std::endl;

                // The mini version doesn't expose a method to directly write to control registers
                // We'll need to rely on writePosition for movement control
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Warning: Servo test failed: " << e.what() << std::endl;
            std::cerr << "Will continue anyway, but arm movement may not work" << std::endl;
        }

        // Create and start the network interface
        std::cout << "Creating network interface in server mode..." << std::endl;
        perseus::ArmNetworkInterfaceMini network(perseus::ArmNetworkInterfaceMini::Mode::Server);
        std::cout << "Starting network interface on port " << perseus::DEFAULT_PORT << "..." << std::endl;

        if (network.start())
        {
            std::cout << "Network started successfully, waiting for leader connection" << std::endl;
        }
        else
        {
            std::cerr << "Failed to start network interface" << std::endl;
            return 1;
        }

        // Set up the callback for receiving servo positions
        std::cout << "Setting up position callback..." << std::endl;
        network.setServoCallback([&reader](const perseus::ServoPositionsMessage& msg)
                                 {
            std::cout << "Received position data from leader:" << std::endl;
            for (int i = 0; i < 6; ++i) {
                std::cout << "  Servo " << (i+1) << ": " << msg.positions[i] << std::endl;
                try {
                    reader.writePosition(i + 1, msg.positions[i]);
                } catch (const std::exception& e) {
                    std::cerr << "  Error writing to servo " << (i+1) << ": " << e.what() << std::endl;
                }
            } });

        // Main loop
        std::cout << "Entering main loop. Press Ctrl+C to exit." << std::endl;
        int connectionCheckCounter = 0;

        while (running)
        {
            // Check and report connection status periodically
            if (connectionCheckCounter++ % 40 == 0)
            {  // Every ~4 seconds
                std::cout << "Network status: "
                          << (network.isConnected() ? "CONNECTED" : "WAITING FOR CONNECTION")
                          << std::endl;
            }

            // Sleep to prevent CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Shutting down follower..." << std::endl;

        // Stop the network
        network.stop();
        std::cout << "Follower shutdown complete" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}