// follower-main-mini.cpp - robust version
#include <chrono>
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

int main(int argc, char* argv[])
{
    // Set up signal handling
    signal(SIGINT, signalHandler);

    std::string serial_port = "/dev/ttyACM0";
    if (argc > 1)
    {
        serial_port = argv[1];
    }

    try
    {
        std::cout << "Starting follower program..." << std::endl;
        std::cout << "Using serial port: " << serial_port << std::endl;

        // Opening the serial port with timeout handling
        std::cout << "Opening serial port " << serial_port << "..." << std::flush;

        ST3215ServoReaderMini* reader = nullptr;
        try
        {
            reader = new ST3215ServoReaderMini(serial_port, 115200);
            reader_ptr = reader;
            std::cout << " success!" << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout << " failed: " << e.what() << std::endl;
            std::cout << "WARNING: Will continue without servo control!" << std::endl;
        }

        // Test servo communication if we have a reader
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
                std::cout << "Possible issues:" << std::endl;
                std::cout << "  - Servo power may be off" << std::endl;
                std::cout << "  - Baud rate might be incorrect (currently using 115200)" << std::endl;
                std::cout << "  - USB connection may be unstable" << std::endl;
                std::cout << "Will continue anyway, but servo control may not work" << std::endl;
            }
        }

        // Create network interface in server mode (listening)
        std::cout << "Creating network interface in server mode..." << std::endl;
        perseus::ArmNetworkInterfaceMini network(perseus::ArmNetworkInterfaceMini::Mode::Server);

        // Set up the callback for receiving servo positions
        if (reader)
        {
            network.setServoCallback([reader](const perseus::ServoPositionsMessage& msg)
                                     {
                std::cout << "Received position data from leader:" << std::endl;
                for (int i = 0; i < 6; ++i) {
                    std::cout << "  Servo " << (i+1) << ": " << msg.positions[i] << std::endl;
                    try {
                        reader->writePosition(i + 1, msg.positions[i]);
                    } catch (const std::exception& e) {
                        std::cerr << "  Error writing to servo " << (i+1) << ": " << e.what() << std::endl;
                    }
                } });
        }
        else
        {
            network.setServoCallback([](const perseus::ServoPositionsMessage& msg)
                                     {
                std::cout << "Received position data from leader (no servo control available):" << std::endl;
                for (int i = 0; i < 6; ++i) {
                    std::cout << "  Servo " << (i+1) << ": " << msg.positions[i] << std::endl;
                } });
        }

        // Start the network interface
        std::cout << "Starting network interface on port " << perseus::DEFAULT_PORT << "..." << std::flush;
        bool network_started = false;
        try
        {
            if (network.start())
            {
                std::cout << " success!" << std::endl;
                network_started = true;
            }
            else
            {
                std::cout << " failed!" << std::endl;
                throw std::runtime_error("Failed to start network interface");
            }
        }
        catch (const std::exception& e)
        {
            std::cout << " failed: " << e.what() << std::endl;
            std::cout << "WARNING: Network listening failed. Check if port " << perseus::DEFAULT_PORT
                      << " is already in use or blocked by firewall." << std::endl;
            // We'll continue anyway for diagnostic purposes
        }

        // Main loop
        std::cout << "Entering main loop. Press Ctrl+C to exit." << std::endl;
        int connectionCheckCounter = 0;

        while (running)
        {
            // Check and report connection status periodically
            if (connectionCheckCounter++ % 40 == 0)
            {  // Every ~4 seconds
                std::cout << "Network status: "
                          << (network_started ? (network.isConnected() ? "CONNECTED TO LEADER" : "WAITING FOR LEADER CONNECTION") : "NOT LISTENING (NETWORK ERROR)")
                          << std::endl;

                if (reader)
                {
                    std::cout << "Servo status: AVAILABLE" << std::endl;
                }
                else
                {
                    std::cout << "Servo status: NOT AVAILABLE" << std::endl;
                }
            }

            // Sleep to prevent CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Shutting down follower..." << std::endl;

        // Clean up
        if (network_started)
        {
            network.stop();
        }

        if (reader)
        {
            delete reader;
            reader_ptr = nullptr;
        }

        std::cout << "Follower shutdown complete" << std::endl;
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