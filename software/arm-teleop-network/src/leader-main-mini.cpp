// leader-main-mini.cpp - robust version
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

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

// Helper function to find available serial ports
std::vector<std::string> getAvailableSerialPorts()
{
    std::vector<std::string> ports;

    // On Linux, try common serial device patterns
    const std::string patterns[] = {
        "/dev/ttyACM",
        "/dev/ttyUSB",
        "/dev/ttyS"};

    for (const auto& pattern : patterns)
    {
        for (int i = 0; i < 10; i++)
        {
            std::string port = pattern + std::to_string(i);
            FILE* file = fopen(port.c_str(), "r");
            if (file)
            {
                fclose(file);
                ports.push_back(port);
            }
        }
    }

    return ports;
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
    else
    {
        // Try to find available ports
        std::cout << "No serial port specified, searching for available ports..." << std::endl;
        auto ports = getAvailableSerialPorts();

        if (ports.empty())
        {
            std::cout << "No serial ports found. Using default: " << serial_port << std::endl;
        }
        else if (ports.size() == 1)
        {
            serial_port = ports[0];
            std::cout << "Found one serial port: " << serial_port << std::endl;
        }
        else
        {
            std::cout << "Found multiple serial ports:" << std::endl;
            for (size_t i = 0; i < ports.size(); i++)
            {
                std::cout << i + 1 << ": " << ports[i] << std::endl;
            }

            std::cout << "Enter the number of the port to use (1-" << ports.size() << "): ";
            int selection;
            std::cin >> selection;

            if (selection > 0 && selection <= static_cast<int>(ports.size()))
            {
                serial_port = ports[selection - 1];
            }
            else
            {
                std::cout << "Invalid selection, using default: " << serial_port << std::endl;
            }
        }
    }

    try
    {
        std::cout << "Starting leader program..." << std::endl;
        std::cout << "Using serial port: " << serial_port << std::endl;
        std::cout << "Target follower IP: " << follower_ip << std::endl;

        // Initialize servo reader
        std::cout << "Opening serial port..." << std::flush;
        ST3215ServoReaderMini* reader = nullptr;

        try
        {
            reader = new ST3215ServoReaderMini(serial_port, 1000000);  // Using 1Mbps as specified for SO-100
            reader_ptr = reader;
            std::cout << " success!" << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout << " failed: " << e.what() << std::endl;
            std::cout << "WARNING: Will continue without servo control!" << std::endl;
        }

        // Test servo communication
        if (reader)
        {
            std::cout << "Testing servo communication..." << std::endl;
            bool any_servo_responded = false;

            for (int i = 1; i <= 6; i++)
            {
                try
                {
                    std::cout << "  Attempting to read position from servo " << i << "..." << std::flush;
                    uint16_t pos = reader->readPosition(i);
                    std::cout << " success! Position: " << pos << std::endl;
                    any_servo_responded = true;
                }
                catch (const std::exception& e)
                {
                    std::cout << " failed: " << e.what() << std::endl;
                }
                // Add a small delay between servo queries
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (any_servo_responded)
            {
                std::cout << "Servo communication test: At least one servo responded successfully" << std::endl;
            }
            else
            {
                std::cout << "WARNING: No servos responded to position queries" << std::endl;
                std::cout << "Will continue anyway, but servo reading may not work" << std::endl;
            }
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

            // Only try to send data if connected and have a reader
            if (network && network->isConnected() && reader)
            {
                perseus::ServoPositionsMessage msg;
                msg.header = {perseus::PROTOCOL_VERSION, perseus::MessageType::SERVO_POSITIONS, 12};

                try
                {
                    // Read positions from all 6 servos
                    bool any_success = false;
                    for (int i = 0; i < 6; ++i)
                    {
                        try
                        {
                            msg.positions[i] = reader->readPosition(i + 1);
                            any_success = true;
                        }
                        catch (const std::exception& e)
                        {
                            // If we can't read a position, use the last known position
                            // or a default value if we have no history
                            if (connectionCheckCounter % 100 == 0)
                            {  // Only log occasionally
                                std::cerr << "Error reading servo " << (i + 1) << ": " << e.what() << std::endl;
                            }
                        }
                    }

                    // Only print positions every ~2 seconds to avoid console spam
                    if (connectionCheckCounter % 20 == 0 && any_success)
                    {
                        std::cout << "Current servo positions: ";
                        for (int i = 0; i < 6; ++i)
                        {
                            std::cout << msg.positions[i] << " ";
                        }
                        std::cout << std::endl;
                    }

                    // Send the positions to the follower if we had at least one successful read
                    if (any_success)
                    {
                        network->sendServoPositions(msg);
                        failedReadCounter = 0;  // Reset failure counter on success
                    }
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
            else if (network && network->isConnected())
            {
                // No reader but we're connected, send a dummy message to keep the connection alive
                if (connectionCheckCounter % 100 == 0)
                {  // Every ~10 seconds
                    std::cout << "No servo reader available, but network is connected" << std::endl;
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

        if (reader)
        {
            delete reader;
            reader_ptr = nullptr;
        }

        std::cout << "Leader shutdown complete" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        if (reader_ptr)
        {
            delete reader_ptr;
            reader_ptr = nullptr;
        }
        return 1;
    }

    return 0;
}