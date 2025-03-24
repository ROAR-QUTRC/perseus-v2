#ifndef PERSEUS_ARM_ASIO_INCLUDES
#define PERSEUS_ARM_ASIO_INCLUDES
#include "perseus-arm-teleop.hpp"
#endif

#include <curses.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include "arm-network.hpp"
#include "arm-servo-protocol.hpp"

// Constants
const int16_t SERVO_REFRESH_DELAY_MS = 25;
const int16_t TORQUE_SAFETY_THRESHOLD = 800;  // 80% of maximum torque

// Global variables for cleanup
static ST3215ServoReader* reader_ptr = nullptr;
static WINDOW* ncurses_win = nullptr;
static perseus::ArmNetworkInterface* network_ptr = nullptr;

static std::atomic<bool> running(true);
static std::atomic<bool> torque_protection(true);  // Default to ON for follower

// Structure to hold servo data
struct ServoData
{
    uint16_t current = 0;
    uint16_t min = 0;
    uint16_t max = 4095;
    int16_t torque = 0;
    std::string error;
    bool mirroring = false;
};

// Find available serial ports
std::vector<std::string> findSerialPorts()
{
    std::vector<std::string> ports;
    const std::filesystem::path dev_path("/dev");

    for (const auto& entry : std::filesystem::directory_iterator(dev_path))
    {
        std::string filename = entry.path().filename().string();
        if (filename.find("ttyUSB") != std::string::npos ||
            filename.find("ttyACM") != std::string::npos)
        {
            ports.push_back(entry.path().string());
        }
    }

    std::sort(ports.begin(), ports.end());
    return ports;
}

// Let user select port for the follower arm
std::string selectSerialPort(const std::vector<std::string>& ports)
{
    if (ports.empty())
    {
        throw std::runtime_error("No serial ports found");
    }

    std::cout << "\nSelect port for Perseus follower arm control\n";
    std::cout << "==========================================\n";
    std::cout << "Available serial ports:\n";
    for (size_t i = 0; i < ports.size(); ++i)
    {
        std::cout << i + 1 << ": " << ports[i] << std::endl;
    }
    std::cout << "0: Rescan for ports\n\n";

    std::string port;
    size_t selection = 0;

    // Select arm port
    while (true)
    {
        std::cout << "Select port for follower arm (0 to rescan, 1-" << ports.size() << " to select): ";
        if (!(std::cin >> selection))
        {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (selection == 0)
        {
            return selectSerialPort(findSerialPorts());  // Rescan and restart
        }

        if (selection > 0 && selection <= ports.size())
        {
            port = ports[selection - 1];
            break;
        }

        std::cout << "Invalid selection. Please try again.\n";
    }

    std::cout << "\nFollower arm will use: " << port << "\n";
    std::cout << "\nPress Enter to continue...";
    std::cin.ignore(10000, '\n');
    std::cin.get();

    return port;
}

// Create a colored progress bar string
void displayProgressBar(WINDOW* win, int y, int x, uint16_t current, uint16_t min, uint16_t max)
{
    // Clamp values to 0-4095
    current = std::min(current, static_cast<uint16_t>(4095));
    min = std::min(min, static_cast<uint16_t>(4095));
    max = std::min(max, static_cast<uint16_t>(4095));

    const size_t barLength = 40;

    // Calculate positions
    size_t currentPos = static_cast<size_t>((static_cast<double>(current) / 4095.0) * barLength);
    size_t minPos = static_cast<size_t>((static_cast<double>(min) / 4095.0) * barLength);
    size_t maxPos = static_cast<size_t>((static_cast<double>(max) / 4095.0) * barLength);

    // Print opening bracket
    mvwaddch(win, y, x, '[');
    x++;

    // Print bar with colors
    for (size_t i = 0; i < barLength; i++)
    {
        if (has_colors())
        {
            if (i == minPos)
            {
                wattron(win, COLOR_PAIR(1));  // Blue for min
                waddch(win, '#');
                wattroff(win, COLOR_PAIR(1));
            }
            else if (i == maxPos)
            {
                wattron(win, COLOR_PAIR(2));  // Green for max
                waddch(win, '#');
                wattroff(win, COLOR_PAIR(2));
            }
            else if (i < currentPos)
            {
                if (i < minPos)
                {
                    wattron(win, COLOR_PAIR(3) | A_DIM);  // Dimmed white for positions before min
                    waddch(win, '#');
                    wattroff(win, COLOR_PAIR(3) | A_DIM);
                }
                else
                {
                    wattron(win, COLOR_PAIR(3));  // Bright white for current valid position
                    waddch(win, '#');
                    wattroff(win, COLOR_PAIR(3));
                }
            }
            else
            {
                waddch(win, ' ');
            }
        }
        else
        {
            // For non-color displays, still show all positions but with different characters
            if (i < currentPos)
            {
                waddch(win, (i < minPos) ? '.' : '#');
            }
            else
            {
                waddch(win, ' ');
            }
        }
    }

    // Print closing bracket
    waddch(win, ']');
}

// Create a colored torque bar string
void displayTorqueBar(WINDOW* win, int y, int x, int16_t torque)
{
    const int width = 5;
    const int16_t max_display = 100;  // Scale display to ±100

    // Scale torque to display width
    int scaled = static_cast<int>((std::abs(static_cast<float>(torque)) / max_display) * width);
    scaled = std::min(scaled, width);

    // Print opening bracket
    mvwaddch(win, y, x, '[');

    if (has_colors())
    {
        for (int i = 0; i < width; i++)
        {
            if (i < scaled)
            {
                int value = std::abs(torque);
                if (value >= 80)
                {
                    wattron(win, COLOR_PAIR(5));  // Red for high torque
                }
                else if (value >= 40)
                {
                    wattron(win, COLOR_PAIR(4));  // Yellow for medium torque
                }
                else
                {
                    wattron(win, COLOR_PAIR(6));  // Grey for low torque
                }
                waddch(win, torque >= 0 ? '+' : '-');
                wattroff(win, COLOR_PAIR(5) | COLOR_PAIR(4) | COLOR_PAIR(6));
            }
            else
            {
                waddch(win, ' ');
            }
        }
    }
    else
    {
        // Non-color display
        for (int i = 0; i < width; i++)
        {
            if (i < scaled)
            {
                waddch(win, torque >= 0 ? '+' : '-');
            }
            else
            {
                waddch(win, ' ');
            }
        }
    }

    // Print closing bracket
    waddch(win, ']');
}

// Display leader status in the UI
void displayLeaderStatus(WINDOW* win, bool connected)
{
    int y = 29;  // Position at the bottom of the display

    mvwprintw(win, y, 0, "Leader Status: ");

    if (has_colors())
    {
        if (connected)
        {
            wattron(win, COLOR_PAIR(2) | A_BOLD);  // Green for connected
            wprintw(win, "CONNECTED");
            wattroff(win, COLOR_PAIR(2) | A_BOLD);
        }
        else
        {
            wattron(win, COLOR_PAIR(5) | A_BOLD);  // Red for disconnected
            wprintw(win, "DISCONNECTED");
            wattroff(win, COLOR_PAIR(5) | A_BOLD);
        }
    }
    else
    {
        wprintw(win, connected ? "CONNECTED" : "DISCONNECTED");
    }
}

// Display servo values in ncurses window
void displayServoValues(WINDOW* win,
                        const std::vector<ServoData>& arm_data,
                        bool leader_connected)
{
    werase(win);

    // Display header
    mvwprintw(win, 0, 0, "Perseus Follower Arm - Network Control Mode");
    mvwprintw(win, 1, 0, "--------------------------------------------------------");

    // Column headers
    mvwprintw(win, 2, 2, "Servo    Current    Min      Max      Torque  Range");
    mvwprintw(win, 3, 0, "--------------------------------------------------------");

    // Display follower arm's servos
    mvwprintw(win, 4, 0, "Follower Arm:");
    for (size_t i = 0; i < 6; ++i)
    {
        int row = i + 5;
        const auto& servo = arm_data[i];

        if (servo.error.empty())
        {
            // Enable yellow highlighting for mirrored servos
            if (servo.mirroring && has_colors())
            {
                wattron(win, COLOR_PAIR(4) | A_BOLD);
            }

            mvwprintw(win, row, 2, "%-8d %8u  %8u  %8u  [Torque: %5d] ",
                      static_cast<int>(i + 1),
                      servo.current,
                      servo.min,
                      servo.max,
                      servo.torque);

            // Display torque bar
            displayTorqueBar(win, row, 42, servo.torque);

            // Display position bar
            displayProgressBar(win, row, 49, servo.current, servo.min, servo.max);

            // Disable yellow highlighting after printing
            if (servo.mirroring && has_colors())
            {
                wattroff(win, COLOR_PAIR(4) | A_BOLD);
            }
        }
        else
        {
            mvwprintw(win, row, 2, "%d: Error: %s",
                      static_cast<int>(i + 1),
                      servo.error.c_str());
        }
    }

    mvwprintw(win, 11, 0, "--------------------------------------------------------");

    // Add instructions and status
    mvwprintw(win, 15, 0, "Instructions:");
    mvwprintw(win, 16, 0, "1. This is a follower arm waiting for commands from the leader");
    mvwprintw(win, 17, 0, "2. Press 't' to toggle torque protection (%s)",
              torque_protection.load() ? "ON" : "OFF");
    mvwprintw(win, 18, 0, "3. Press 'r' to reset mirroring for all servos");
    mvwprintw(win, 19, 0, "4. Press 'c' to change listen port (currently %d)", perseus::DEFAULT_PORT);
    mvwprintw(win, 20, 0, "5. Press Ctrl+C to exit");

    // Add color legend if colors are available
    if (has_colors())
    {
        mvwprintw(win, 22, 0, "Legend: ");
        wattron(win, COLOR_PAIR(4) | A_BOLD);
        wprintw(win, "Yellow rows = Mirrored servos");
        wattroff(win, COLOR_PAIR(4) | A_BOLD);
    }

    // Display leader connection status
    displayLeaderStatus(win, leader_connected);

    wrefresh(win);
}

// Scale a position value from one range to another
uint16_t scalePosition(uint16_t pos, uint16_t srcMin, uint16_t srcMax, uint16_t destMin, uint16_t destMax)
{
    // Ensure we don't divide by zero
    if (srcMax == srcMin)
        return destMin;

    // Calculate the scaling factor and apply it
    double scale = static_cast<double>(destMax - destMin) / static_cast<double>(srcMax - srcMin);
    return static_cast<uint16_t>(destMin + (pos - srcMin) * scale);
}

// Change the follower's listen port
bool changeListenPort(perseus::ArmNetworkInterface** network_ptr_ref, WINDOW* win)
{
    // If already have a network interface, clean it up
    if (*network_ptr_ref)
    {
        delete *network_ptr_ref;
        *network_ptr_ref = nullptr;
    }

    // Get port from user
    int port = perseus::DEFAULT_PORT;

    // Temporarily disable nodelay for user input
    nodelay(win, FALSE);

    // Clear a space for the dialog
    for (int i = 15; i < 23; i++)
    {
        mvwprintw(win, i, 0, "                                                                                ");
    }

    mvwprintw(win, 15, 0, "Enter listen port number [%d]: ", perseus::DEFAULT_PORT);
    char port_str[10] = {0};
    echo();
    wgetnstr(win, port_str, sizeof(port_str) - 1);
    noecho();

    // If port string is not empty, parse it
    if (port_str[0] != '\0')
    {
        try
        {
            port = std::stoi(port_str);
        }
        catch (...)
        {
            mvwprintw(win, 16, 0, "Invalid port number, using default: %d", perseus::DEFAULT_PORT);
            wrefresh(win);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            port = perseus::DEFAULT_PORT;
        }
    }

    // Restore nodelay for the main loop
    nodelay(win, TRUE);

    try
    {
        mvwprintw(win, 16, 0, "Starting network server on port %d...", port);
        wrefresh(win);

        // Create the network interface in server mode (just port, no host)
        *network_ptr_ref = new perseus::ArmNetworkInterface(port);

        // Start the network interface
        if (!(*network_ptr_ref)->start())
        {
            throw std::runtime_error("Failed to start network interface");
        }

        mvwprintw(win, 16, 0, "Network server started on port %d", port);
        wrefresh(win);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return true;
    }
    catch (const std::exception& e)
    {
        mvwprintw(win, 16, 0, "Failed to start network server: %s", e.what());
        wrefresh(win);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Clean up if starting failed
        if (*network_ptr_ref)
        {
            delete *network_ptr_ref;
            *network_ptr_ref = nullptr;
        }

        return false;
    }
}

// Disable servo torque and clean up resources before exit
void disableTorqueAndCleanup()
{
    // First disable ncurses if it's active
    if (ncurses_win != nullptr)
    {
        mvwprintw(ncurses_win, 25, 0, "Disabling servo torque...");
        wrefresh(ncurses_win);
    }

    // Disable torque for all servos on the follower arm
    if (reader_ptr != nullptr)
    {
        for (uint8_t i = 1; i <= 6; ++i)
        {
            try
            {
                reader_ptr->writeControlRegister(i, 0x28, 0);  // 0x28 is torque enable register
            }
            catch (...)
            {
                // Ignore errors during shutdown
            }
        }
    }

    // Clean up network interface
    if (network_ptr != nullptr)
    {
        network_ptr->stop();
        delete network_ptr;
        network_ptr = nullptr;
    }

    // Clean up ncurses
    if (ncurses_win != nullptr)
    {
        endwin();
        ncurses_win = nullptr;
    }
}

// Signal handler for Ctrl+C
void signalHandler([[maybe_unused]] int signum)
{
    running = false;
    disableTorqueAndCleanup();
}

// Main function
int main(int argc, char* argv[])
{
    std::vector<ServoData> arm_data(6);
    try
    {
        // Set up signal handling
        signal(SIGINT, signalHandler);

        // Get follower arm port path
        std::string port_path;
        if (argc > 1)
        {
            port_path = argv[1];
        }
        else
        {
            auto available_ports = findSerialPorts();
            port_path = selectSerialPort(available_ports);
        }

        std::cout << "Using serial port for follower arm: " << port_path << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Initialize ncurses
        ncurses_win = initscr();
        cbreak();
        noecho();
        curs_set(0);
        nodelay(ncurses_win, TRUE);
        keypad(ncurses_win, TRUE);

        // Initialize colors if terminal supports them
        if (has_colors())
        {
            start_color();
            init_pair(1, COLOR_BLUE, COLOR_BLACK);    // For min value
            init_pair(2, COLOR_GREEN, COLOR_BLACK);   // For max value
            init_pair(3, COLOR_WHITE, COLOR_BLACK);   // For current value
            init_pair(4, COLOR_YELLOW, COLOR_BLACK);  // For mirrored servos
            init_pair(5, COLOR_RED, COLOR_BLACK);     // For high torque/errors
            init_pair(6, COLOR_WHITE, COLOR_BLACK);   // For low torque (grey)
        }

        // Initialize servo reader for follower arm
        ST3215ServoReader reader(port_path, 1000000, 30);  // 30 sets the acceleration (magic number)
        reader_ptr = &reader;

        // Set up network interface for receiving commands
        uint16_t listen_port = (argc > 2) ? static_cast<uint16_t>(std::stoi(argv[2])) : perseus::DEFAULT_PORT;
        network_ptr = new perseus::ArmNetworkInterface(listen_port);

        // Set up callbacks for network messages
        network_ptr->setServoPositionsCallback([&arm_data, &reader](const perseus::ServoPositionsMessage& message)
                                               {
            for (int i = 0; i < 6; i++) {
                try {
                    if (arm_data[i].mirroring) {
                        // Update our data structure
                        arm_data[i].current = message.servos[i].position;
                        arm_data[i].torque = message.servos[i].torque;
                        
                        // Send to physical servo
                        reader.writePosition(i + 1, message.servos[i].position);
                    }
                } catch (const std::exception& e) {
                    arm_data[i].error = std::string("Mirror error: ") + e.what();
                    arm_data[i].mirroring = false;
                }
            } });

        network_ptr->setServoMirroringCallback([&arm_data, &reader](const perseus::ServoMirroringMessage& message)
                                               {
            if (message.servo_id >= 1 && message.servo_id <= 6) {
                size_t idx = message.servo_id - 1;
                arm_data[idx].mirroring = message.mirroring;
                
                try {
                    // Enable/disable torque based on mirroring state
                    reader.writeControlRegister(message.servo_id, 0x28, message.mirroring ? 1 : 0);
                    
                    if (!message.mirroring) {
                        arm_data[idx].error.clear();
                    }
                } catch (const std::exception& e) {
                    arm_data[idx].error = std::string("Torque control error: ") + e.what();
                }
            } });

        network_ptr->setCalibrationCallback([&arm_data](const perseus::CalibrationMessage& message)
                                            {
            for (int i = 0; i < 6; i++) {
                arm_data[i].min = message.servos[i].min;
                arm_data[i].max = message.servos[i].max;
            } });

        // Start the network interface
        if (!network_ptr->start())
        {
            throw std::runtime_error("Failed to start network interface on port " + std::to_string(listen_port));
        }

        // Main loop
        while (running)
        {
            // Read all servo positions and torque values from follower arm
            for (uint8_t i = 0; i < 6; ++i)
            {
                try
                {
                    // Always read current position for all servos
                    uint16_t current_position = reader.readPosition(i + 1);
                    arm_data[i].current = current_position;

                    // Read torque for all servos
                    int16_t torque = reader.readLoad(i + 1);
                    arm_data[i].torque = torque;

                    // Check if torque exceeds safety threshold for mirrored servos
                    if (arm_data[i].mirroring && torque_protection.load() && std::abs(torque) > TORQUE_SAFETY_THRESHOLD)
                    {
                        // Disable torque
                        reader.writeControlRegister(i + 1, 0x28, 0);  // 0x28 is torque enable register
                        arm_data[i].error = "Torque limit exceeded - disabled";
                        arm_data[i].mirroring = false;

                        // Send status to leader if connected
                        if (network_ptr && network_ptr->isConnected())
                        {
                            perseus::StatusMessage status;
                            status.header.protocol_version = perseus::PROTOCOL_VERSION;
                            status.header.type = perseus::MessageType::STATUS_INFO;
                            status.status = perseus::ArmStatus::ERROR_TORQUE;
                            status.error_servo_id = i + 1;
                            status.error_message = arm_data[i].error;

                            network_ptr->sendStatus(status);
                        }
                    }
                    else if (!arm_data[i].error.empty())
                    {
                        arm_data[i].error.clear();
                    }
                }
                catch (const std::exception& e)
                {
                    if (arm_data[i].mirroring)
                    {
                        arm_data[i].error = e.what();
                        arm_data[i].mirroring = false;
                        arm_data[i].torque = 0;

                        // Send error status to leader
                        if (network_ptr && network_ptr->isConnected())
                        {
                            perseus::StatusMessage status;
                            status.header.protocol_version = perseus::PROTOCOL_VERSION;
                            status.header.type = perseus::MessageType::STATUS_INFO;
                            status.status = perseus::ArmStatus::ERROR_COMMUNICATION;
                            status.error_servo_id = i + 1;
                            status.error_message = arm_data[i].error;

                            network_ptr->sendStatus(status);
                        }
                    }
                    else
                    {
                        arm_data[i].error = e.what();
                        arm_data[i].current = 0;  // Reset current on error
                        arm_data[i].torque = 0;   // Reset torque on error
                    }
                }
            }

            // Update display with arm data and network status
            displayServoValues(ncurses_win,
                               arm_data,
                               network_ptr ? network_ptr->isConnected() : false);

            // Handle keyboard input
            int ch = wgetch(ncurses_win);
            if (ch == 't' || ch == 'T')
            {
                bool new_state = !torque_protection.load();
                torque_protection.store(new_state);

                mvwprintw(ncurses_win, 26, 0, "                                                                        ");
                if (has_colors())
                {
                    wattron(ncurses_win, COLOR_PAIR(new_state ? 2 : 5) | A_BOLD);
                }
                mvwprintw(ncurses_win, 26, 0, "Torque protection %s", new_state ? "ENABLED" : "DISABLED");
                if (has_colors())
                {
                    wattroff(ncurses_win, COLOR_PAIR(new_state ? 2 : 5) | A_BOLD);
                }
                wrefresh(ncurses_win);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                mvwprintw(ncurses_win, 26, 0, "                                                                        ");
            }
            else if (ch == 'r' || ch == 'R')
            {
                // Reset all mirroring
                mvwprintw(ncurses_win, 26, 0, "Resetting all servo mirroring...");
                wrefresh(ncurses_win);

                for (size_t i = 0; i < arm_data.size(); ++i)
                {
                    try
                    {
                        // Disable torque for all servos
                        reader.writeControlRegister(i + 1, 0x28, 0);
                        arm_data[i].mirroring = false;
                        arm_data[i].error.clear();
                    }
                    catch (const std::exception& e)
                    {
                        arm_data[i].error = std::string("Reset error: ") + e.what();
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                mvwprintw(ncurses_win, 26, 0, "                                                                        ");
            }
            else if (ch == 'c' || ch == 'C')
            {
                // Change listen port
                changeListenPort(&network_ptr, ncurses_win);
            }

            // Delay to prevent overwhelming servos
            std::this_thread::sleep_for(std::chrono::milliseconds(SERVO_REFRESH_DELAY_MS));
        }

        // Clean up
        disableTorqueAndCleanup();
        std::cout << "Program terminated by user." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        disableTorqueAndCleanup();
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}