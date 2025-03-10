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
static std::atomic<bool> torque_protection(false);

// Structure to hold servo data
struct ServoData
{
    uint16_t current = 0;
    uint16_t min = 4095;
    uint16_t max = 0;
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

// Let user select port for the leader arm
std::string selectSerialPort(const std::vector<std::string>& ports)
{
    if (ports.empty())
    {
        throw std::runtime_error("No serial ports found");
    }

    std::cout << "\nSelect port for Perseus leader arm control\n";
    std::cout << "========================================\n";
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
        std::cout << "Select port for leader arm (0 to rescan, 1-" << ports.size() << " to select): ";
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

    std::cout << "\nLeader arm will use: " << port << "\n";
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

// Display follower status and IP address in the UI
void displayFollowerStatus(WINDOW* win, bool connected, const std::string& ip_address = "")
{
    int y = 29;  // Position at the bottom of the display

    mvwprintw(win, y, 0, "Follower Status: ");

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

    // Display follower IP address on the line after the servo table
    // Display follower IP address on the line after the servo table
    mvwprintw(win, 12, 0, "Follower: ");

    if (has_colors())
    {
        if (connected && !ip_address.empty())
        {
            wattron(win, COLOR_PAIR(2) | A_BOLD);  // Green for connected IP
            wprintw(win, "%s", ip_address.c_str());
            wattroff(win, COLOR_PAIR(2) | A_BOLD);
        }
        else
        {
            wattron(win, COLOR_PAIR(5) | A_BOLD);  // Red for not connected
            wprintw(win, "N/C");
            wattroff(win, COLOR_PAIR(5) | A_BOLD);
        }
    }
    else
    {
        wprintw(win, "%s", connected ? ip_address.c_str() : "N/C");
    }
}

// Display servo values in ncurses window
void displayServoValues(WINDOW* win,
                        const std::vector<ServoData>& arm_data,
                        bool follower_connected,
                        const std::string& follower_ip_address = "")
{
    werase(win);

    // Display header
    mvwprintw(win, 0, 0, "Perseus Leader Arm - Network Control Mode");
    mvwprintw(win, 1, 0, "--------------------------------------------------------");

    // Column headers
    mvwprintw(win, 2, 2, "Servo    Current    Min      Max      Torque  Range");
    mvwprintw(win, 3, 0, "--------------------------------------------------------");

    // Display leader arm's servos
    mvwprintw(win, 4, 0, "Leader Arm:");
    for (size_t i = 0; i < 6; ++i)
    {
        int row = i + 5;
        const auto& servo = arm_data[i];

        if (servo.error.empty())
        {
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
        }
        else
        {
            mvwprintw(win, row, 2, "%d: Error: %s",
                      static_cast<int>(i + 1),
                      servo.error.c_str());
        }
    }

    mvwprintw(win, 11, 0, "--------------------------------------------------------");

    // Add instructions and working directory
    mvwprintw(win, 15, 0, "Instructions:");
    mvwprintw(win, 16, 0, "1. Move the leader arm through its full range of motion");
    mvwprintw(win, 17, 0, "2. Press 's' to save calibration");
    mvwprintw(win, 18, 0, "3. Press keys 1-6 to toggle mirroring for each servo");
    mvwprintw(win, 19, 0, "4. Press 't' to toggle torque protection (%s)",
              torque_protection.load() ? "ON" : "OFF");
    mvwprintw(win, 20, 0, "5. Press 'l' to load calibration from file");
    mvwprintw(win, 21, 0, "6. Press 'c' to connect to a different follower");
    mvwprintw(win, 22, 0, "7. Press Ctrl+C to exit");

    // Add color legend if colors are available
    if (has_colors())
    {
        mvwprintw(win, 24, 0, "Legend: ");
        wattron(win, COLOR_PAIR(4) | A_BOLD);
        wprintw(win, "Yellow rows = Mirrored servos");
        wattroff(win, COLOR_PAIR(4) | A_BOLD);
    }

    // Display follower connection status
    displayFollowerStatus(win, follower_connected, follower_connected ? follower_ip_address : "");

    wrefresh(win);
}

// Export calibration data to a YAML file
void exportCalibrationData(const std::vector<ServoData>& arm_data,
                           const std::string& port)
{
    try
    {
        YAML::Node config;

        // Add metadata including timestamp
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
        config["timestamp"] = ss.str();

        // Add port information
        config["leader_port"] = port;

        // Add calibration data for leader arm
        YAML::Node arm_node;
        for (size_t i = 0; i < arm_data.size(); ++i)
        {
            YAML::Node servo;
            servo["id"] = i + 1;
            servo["min"] = arm_data[i].min;
            servo["max"] = arm_data[i].max;
            servo["mirroring"] = arm_data[i].mirroring;
            arm_node["servos"].push_back(servo);
        }
        config["leader_arm"] = arm_node;

        // Fixed filename for consistency
        const std::string filename = "arm_calibration.yaml";

        // Open file with overwrite permissions
        std::ofstream fout(filename, std::ios::out | std::ios::trunc);
        if (!fout.is_open())
        {
            throw std::runtime_error("Failed to open file for writing: " + filename);
        }

        // Write configuration with error checking
        fout << config;
        if (fout.fail())
        {
            fout.close();
            throw std::runtime_error("Failed to write data to file: " + filename);
        }

        // Ensure all data is written and close file
        fout.flush();
        if (fout.fail())
        {
            fout.close();
            throw std::runtime_error("Failed to flush data to file: " + filename);
        }
        fout.close();

        // Check for any errors that occurred during close
        if (fout.fail())
        {
            throw std::runtime_error("Error occurred while closing file: " + filename);
        }
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(std::string("YAML error while saving calibration: ") + e.what());
    }
    catch (const std::exception& e)
    {
        throw;  // Re-throw other exceptions
    }
}

// Load calibration data from a YAML file
void loadCalibrationData(std::vector<ServoData>& arm_data,
                         const std::string& filename = "arm_calibration.yaml")
{
    try
    {
        YAML::Node config = YAML::LoadFile(filename);

        // Load leader arm servo data
        if (config["leader_arm"] && config["leader_arm"]["servos"])
        {
            auto arm_servos = config["leader_arm"]["servos"];
            for (size_t i = 0; i < std::min(arm_data.size(), arm_servos.size()); ++i)
            {
                auto servo = arm_servos[i];
                if (servo["min"] && servo["max"])
                {
                    arm_data[i].min = servo["min"].as<uint16_t>();
                    arm_data[i].max = servo["max"].as<uint16_t>();
                    // Ensure current position stays within new bounds
                    arm_data[i].current = std::max(arm_data[i].min,
                                                   std::min(arm_data[i].max,
                                                            arm_data[i].current));
                }
                if (servo["mirroring"])
                {
                    arm_data[i].mirroring = servo["mirroring"].as<bool>();
                }
            }
        }

        // Share the calibration data over the network if connected
        if (network_ptr && network_ptr->isConnected())
        {
            perseus::CalibrationMessage message;
            message.header.protocol_version = perseus::PROTOCOL_VERSION;
            message.header.type = perseus::MessageType::CALIBRATION_DATA;
            message.header.payload_length = sizeof(perseus::CalibrationMessage::ServoCalibration) * 6;

            for (size_t i = 0; i < arm_data.size(); ++i)
            {
                message.servos[i].min = arm_data[i].min;
                message.servos[i].max = arm_data[i].max;
            }

            network_ptr->sendCalibration(message);
        }
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(std::string("YAML error while loading calibration: ") + e.what());
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(std::string("Error loading calibration: ") + e.what());
    }
}

// Connect to a follower arm over the network
bool connectToFollower(perseus::ArmNetworkInterface** network_ptr_ref, WINDOW* win, std::string& follower_ip)
{
    // If already have a network interface, clean it up
    if (*network_ptr_ref)
    {
        delete *network_ptr_ref;
        *network_ptr_ref = nullptr;
    }

    // Get IP address and port from user
    char ip_address[64] = {0};
    int port = perseus::DEFAULT_PORT;

    // Temporarily disable nodelay for user input
    nodelay(win, FALSE);

    // Clear a space for the dialog
    for (int i = 15; i < 25; i++)
    {
        mvwprintw(win, i, 0, "                                                                                ");
    }

    mvwprintw(win, 15, 0, "Enter follower IP address: ");
    echo();
    wgetnstr(win, ip_address, sizeof(ip_address) - 1);
    noecho();

    mvwprintw(win, 16, 0, "Enter port number [%d]: ", perseus::DEFAULT_PORT);
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
            mvwprintw(win, 17, 0, "Invalid port number, using default: %d", perseus::DEFAULT_PORT);
            wrefresh(win);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            port = perseus::DEFAULT_PORT;
        }
    }

    // Restore nodelay for the main loop
    nodelay(win, TRUE);

    // Validate IP address
    if (std::string(ip_address).empty())
    {
        mvwprintw(win, 17, 0, "IP address cannot be empty");
        wrefresh(win);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return false;
    }

    try
    {
        mvwprintw(win, 17, 0, "Connecting to %s:%d...", ip_address, port);
        wrefresh(win);

        // Create the network interface
        *network_ptr_ref = new perseus::ArmNetworkInterface(ip_address, port);

        // Start the network interface
        if (!(*network_ptr_ref)->start())
        {
            throw std::runtime_error("Failed to start network interface");
        }

        // Wait up to 5 seconds for connection
        for (int i = 0; i < 50; i++)
        {
            if ((*network_ptr_ref)->isConnected())
            {
                mvwprintw(win, 17, 0, "Connected to follower at %s:%d", ip_address, port);
                wrefresh(win);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                follower_ip = ip_address;  // Store the IP address
                return true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Update progress indicator
            mvwprintw(win, 17, 0, "Connecting to %s:%d... %c",
                      ip_address, port, "|/-\\"[i % 4]);
            wrefresh(win);
        }

        throw std::runtime_error("Connection timed out");
    }
    catch (const std::exception& e)
    {
        mvwprintw(win, 17, 0, "Connection failed: %s", e.what());
        wrefresh(win);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Clean up if connection failed
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
        mvwprintw(ncurses_win, 27, 0, "Disabling servo torque...");
        wrefresh(ncurses_win);
    }

    // Disable torque for all servos on the leader arm
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
    std::string follower_ip_address = "";  // Store the follower IP address
    try
    {
        // Set up signal handling
        signal(SIGINT, signalHandler);

        // Get leader arm port path
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

        std::cout << "Using serial port for leader arm: " << port_path << std::endl;
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

        // Initialize servo reader for leader arm
        ST3215ServoReader reader(port_path, 1000000, 30);  // 30 sets the acceleration (magic number)
        reader_ptr = &reader;

        // Main loop
        while (running)
        {
            // Read all servo positions and torque from leader arm
            for (uint8_t i = 0; i < 6; ++i)
            {
                try
                {
                    auto& servo = arm_data[i];
                    servo.current = reader.readPosition(i + 1);
                    servo.min = std::min(servo.min, servo.current);
                    servo.max = std::max(servo.max, servo.current);

                    // Read torque value
                    int16_t torque = reader.readLoad(i + 1);
                    servo.torque = torque;

                    // Check if torque exceeds safety threshold
                    if (torque_protection.load() && std::abs(torque) > TORQUE_SAFETY_THRESHOLD)
                    {
                        // Disable torque
                        reader.writeControlRegister(i + 1, 0x28, 0);  // 0x28 is torque enable register
                        servo.error = "Torque limit exceeded - disabled";
                    }
                    else if (!servo.error.empty())
                    {
                        servo.error.clear();
                    }
                }
                catch (const std::exception& e)
                {
                    arm_data[i].error = e.what();
                    arm_data[i].torque = 0;  // Clear torque on error
                }
            }

            // Send servo positions to follower over network if connected
            if (network_ptr && network_ptr->isConnected())
            {
                perseus::ServoPositionsMessage message;
                message.header.protocol_version = perseus::PROTOCOL_VERSION;
                message.header.type = perseus::MessageType::SERVO_POSITIONS;
                message.header.payload_length = sizeof(perseus::ServoPositionsMessage::ServoData) * 6;

                for (size_t i = 0; i < arm_data.size(); ++i)
                {
                    message.servos[i].position = arm_data[i].current;
                    message.servos[i].torque = arm_data[i].torque;
                }

                network_ptr->sendServoPositions(message);
            }

            // Update display with arm data and network status
            displayServoValues(ncurses_win,
                               arm_data,
                               network_ptr ? network_ptr->isConnected() : false,
                               follower_ip_address);

            // Handle keyboard input
            int ch = wgetch(ncurses_win);
            if (ch == 't' || ch == 'T')
            {
                bool new_state = !torque_protection.load();
                torque_protection.store(new_state);

                mvwprintw(ncurses_win, 27, 0, "                                                                        ");
                if (has_colors())
                {
                    wattron(ncurses_win, COLOR_PAIR(new_state ? 2 : 5) | A_BOLD);
                }
                mvwprintw(ncurses_win, 27, 0, "Torque protection %s", new_state ? "ENABLED" : "DISABLED");
                if (has_colors())
                {
                    wattroff(ncurses_win, COLOR_PAIR(new_state ? 2 : 5) | A_BOLD);
                }
                wrefresh(ncurses_win);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                mvwprintw(ncurses_win, 27, 0, "                                                                        ");
            }
            else if (ch == 'l' || ch == 'L')
            {
                mvwprintw(ncurses_win, 28, 0, "Loading calibration data...");
                wrefresh(ncurses_win);

                try
                {
                    loadCalibrationData(arm_data);
                    mvwprintw(ncurses_win, 28, 0, "                                                    ");
                    mvwprintw(ncurses_win, 28, 0, "Calibration data loaded successfully!");
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                catch (const std::exception& e)
                {
                    mvwprintw(ncurses_win, 28, 0, "                                                    ");
                    mvwprintw(ncurses_win, 28, 0, "Error loading calibration: %s", e.what());
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }

                mvwprintw(ncurses_win, 28, 0, "                                                    ");
                wrefresh(ncurses_win);
            }
            else if (ch == 's' || ch == 'S')
            {
                mvwprintw(ncurses_win, 28, 0, "Saving calibration data...");
                wrefresh(ncurses_win);

                try
                {
                    exportCalibrationData(arm_data, port_path);
                    mvwprintw(ncurses_win, 28, 0, "                                                                        ");
                    mvwprintw(ncurses_win, 28, 0, "Calibration data saved successfully! Press any key to continue");
                    wrefresh(ncurses_win);

                    // Wait for any key
                    nodelay(ncurses_win, FALSE);  // Switch to blocking mode temporarily
                    wgetch(ncurses_win);
                    nodelay(ncurses_win, TRUE);  // Switch back to non-blocking

                    // Clear status line
                    mvwprintw(ncurses_win, 28, 0, "                                                                        ");
                    wrefresh(ncurses_win);

                    // Share the calibration with follower if connected
                    if (network_ptr && network_ptr->isConnected())
                    {
                        perseus::CalibrationMessage message;
                        message.header.protocol_version = perseus::PROTOCOL_VERSION;
                        message.header.type = perseus::MessageType::CALIBRATION_DATA;
                        message.header.payload_length = sizeof(perseus::CalibrationMessage::ServoCalibration) * 6;

                        for (size_t i = 0; i < arm_data.size(); ++i)
                        {
                            message.servos[i].min = arm_data[i].min;
                            message.servos[i].max = arm_data[i].max;
                        }

                        network_ptr->sendCalibration(message);

                        mvwprintw(ncurses_win, 28, 0, "Calibration data shared with follower");
                        wrefresh(ncurses_win);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        mvwprintw(ncurses_win, 28, 0, "                                                                        ");
                    }
                }
                catch (const std::exception& e)
                {
                    mvwprintw(ncurses_win, 28, 0, "                                                    ");
                    mvwprintw(ncurses_win, 28, 0, "Error saving calibration: %s", e.what());
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }

                mvwprintw(ncurses_win, 28, 0, "                                                    ");
                wrefresh(ncurses_win);
            }
            else if (ch == 'c' || ch == 'C')
            {
                // Connect to a different follower
                connectToFollower(&network_ptr, ncurses_win, follower_ip_address);
            }
            else if (ch >= '1' && ch <= '6')
            {
                // Toggle mirroring for the selected servo
                int servo_idx = ch - '1';
                if (servo_idx >= 0 && servo_idx < 6)
                {
                    // Toggle mirroring state
                    arm_data[servo_idx].mirroring = !arm_data[servo_idx].mirroring;

                    // Send mirroring command to follower if connected
                    if (network_ptr && network_ptr->isConnected())
                    {
                        perseus::ServoMirroringMessage message;
                        message.header.protocol_version = perseus::PROTOCOL_VERSION;
                        message.header.type = perseus::MessageType::SERVO_MIRRORING;
                        message.header.payload_length = 2;
                        message.servo_id = servo_idx + 1;
                        message.mirroring = arm_data[servo_idx].mirroring;

                        network_ptr->sendServoMirroring(message);

                        mvwprintw(ncurses_win, 28, 0, "Servo %d mirroring %s",
                                  servo_idx + 1,
                                  arm_data[servo_idx].mirroring ? "enabled" : "disabled");
                        wrefresh(ncurses_win);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        mvwprintw(ncurses_win, 28, 0, "                                                    ");
                    }
                }
            }

            // Delay to prevent overwhelming servos
            std::this_thread::sleep_for(std::chrono::milliseconds(SERVO_REFRESH_DELAY_MS));
        }  // End of while(running) loop

        // Clean up
        disableTorqueAndCleanup();
        std::cout << "Program terminated by user." << std::endl;
        return 0;
    }  // End of try block
    catch (const std::exception& e)
    {
        disableTorqueAndCleanup();
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    // End of main function
}