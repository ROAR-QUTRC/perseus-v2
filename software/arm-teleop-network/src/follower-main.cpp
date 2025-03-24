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
    uint16_t min = 4095;  // Start with maximum possible value
    uint16_t max = 0;     // Start with minimum possible value
    int16_t torque = 0;
    std::string error;
    bool mirroring = false;
};

// Structure to hold leader calibration data
struct LeaderCalibration
{
    uint16_t min = 0;
    uint16_t max = 4095;
};

// Global array to store leader calibration for mapping
static std::vector<LeaderCalibration> leader_calibration(6);

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
                waddch(win, '|');
                wattroff(win, COLOR_PAIR(1));
            }
            else if (i == maxPos)
            {
                wattron(win, COLOR_PAIR(2));  // Green for max
                waddch(win, '|');
                wattroff(win, COLOR_PAIR(2));
            }
            else if (i < currentPos)
            {
                if (i < minPos || i > maxPos)
                {
                    wattron(win, COLOR_PAIR(3) | A_DIM);  // Dimmed white for positions outside range
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
            // For non-color displays, show positions with different characters
            if (i == minPos)
            {
                waddch(win, '[');
            }
            else if (i == maxPos)
            {
                waddch(win, ']');
            }
            else if (i < currentPos)
            {
                waddch(win, '#');
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
// Update display with arm data and network status
void displayServoValues(WINDOW* win,
                        const std::vector<ServoData>& arm_data,
                        const std::vector<LeaderCalibration>& leader_cal,
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
    // mvwprintw(win, 12, 0, "Leader Calibration: ");
    // for (size_t i = 0; i < 6; ++i)
    //{
    //     int row = i + 13;
    //     const auto& cal = leader_cal[i];
    //     mvwprintw(win, row, 2, "Servo %d:  Min: %4u  Max: %4u",
    //               static_cast<int>(i + 1), cal.min, cal.max);
    // }

    // Add instructions and status
    mvwprintw(win, 15, 0, "Instructions:");
    mvwprintw(win, 16, 0, "1. This is a follower arm waiting for commands from the leader");
    mvwprintw(win, 17, 0, "2. Auto-calibration is always active (updates min/max positions)");
    mvwprintw(win, 18, 0, "3. Press 't' to toggle torque protection (%s)",
              torque_protection.load() ? "ON" : "OFF");
    mvwprintw(win, 19, 0, "4. Press 's' to save calibration");
    mvwprintw(win, 20, 0, "5. Press 'l' to load calibration from file");
    mvwprintw(win, 21, 0, "6. Press 'r' to reset mirroring for all servos");
    mvwprintw(win, 22, 0, "7. Press 'c' to change listen port (currently %d)", perseus::DEFAULT_PORT);
    mvwprintw(win, 23, 0, "8. Press Ctrl+C to exit");

    // Add color legend if colors are available
    if (has_colors())
    {
        mvwprintw(win, 24, 0, "Legend: ");
        wattron(win, COLOR_PAIR(4) | A_BOLD);
        wprintw(win, "Yellow rows = Mirrored servos");
        wattroff(win, COLOR_PAIR(4) | A_BOLD);
    }

    // Display leader connection status
    displayLeaderStatus(win, leader_connected);

    wrefresh(win);
}

void updateServoCalibration(std::vector<ServoData>& arm_data, uint16_t position, size_t servo_index)
{
    // Update min/max based on actual observations
    arm_data[servo_index].min = std::min(arm_data[servo_index].min, position);
    arm_data[servo_index].max = std::max(arm_data[servo_index].max, position);
}

// Scale a position value from one range to another
uint16_t scalePosition(uint16_t pos, uint16_t srcMin, uint16_t srcMax, uint16_t destMin, uint16_t destMax)
{
    // Ensure we don't divide by zero
    if (srcMax == srcMin)
        return destMin;

    // Also handle inverted ranges by checking if max < min
    bool srcInverted = srcMax < srcMin;
    bool destInverted = destMax < destMin;

    // If one range is inverted but not the other, we need to invert the position
    bool needInversion = srcInverted != destInverted;

    // Normalize the ranges for calculation
    uint16_t normalizedSrcMin = srcInverted ? srcMax : srcMin;
    uint16_t normalizedSrcMax = srcInverted ? srcMin : srcMax;
    uint16_t normalizedDestMin = destInverted ? destMax : destMin;
    uint16_t normalizedDestMax = destInverted ? destMin : destMax;

    // Clamp pos to source range to prevent out-of-range values
    pos = std::max(std::min(pos, std::max(srcMin, srcMax)), std::min(srcMin, srcMax));

    // If we need inversion, invert pos within source range
    if (needInversion)
    {
        pos = srcMin + srcMax - pos;
    }

    // Calculate the scaling factor and apply it
    double scale = static_cast<double>(normalizedDestMax - normalizedDestMin) /
                   static_cast<double>(normalizedSrcMax - normalizedSrcMin);

    return static_cast<uint16_t>(normalizedDestMin +
                                 (pos - normalizedSrcMin) * scale);
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

// Export calibration data to a YAML file
void exportCalibrationData(const std::vector<ServoData>& arm_data,
                           const std::vector<LeaderCalibration>& leader_calibration,
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
        config["follower_port"] = port;

        // Add calibration data for follower arm
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
        config["follower_arm"] = arm_node;

        YAML::Node leader_node;
        for (size_t i = 0; i < leader_calibration.size(); ++i)
        {
            YAML::Node servo;
            servo["id"] = i + 1;
            servo["min"] = leader_calibration[i].min;
            servo["max"] = leader_calibration[i].max;
            leader_node["servos"].push_back(servo);
        }
        config["leader_arm"] = leader_node;
        // Fixed filename for consistency
        const std::string filename = "follower_arm_calibration.yaml";

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
                         std::vector<LeaderCalibration>& leader_calibration,
                         const std::string& filename = "follower_arm_calibration.yaml")
{
    try
    {
        mvwprintw(ncurses_win, 26, 0, "Loading calibration from %s...", filename.c_str());
        wrefresh(ncurses_win);

        YAML::Node config = YAML::LoadFile(filename);

        // Temporary storage for tracking which servos need mirroring updates
        std::vector<std::pair<uint8_t, bool>> mirroring_updates;

        // Load follower arm servo data
        if (config["follower_arm"] && config["follower_arm"]["servos"])
        {
            auto arm_servos = config["follower_arm"]["servos"];
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

                // Check if mirroring status is changing
                if (servo["mirroring"])
                {
                    bool new_mirroring = servo["mirroring"].as<bool>();
                    if (arm_data[i].mirroring != new_mirroring)
                    {
                        mirroring_updates.push_back({static_cast<uint8_t>(i + 1), new_mirroring});
                    }
                    arm_data[i].mirroring = new_mirroring;
                }
            }
        }

        // Load leader arm calibration data
        if (config["leader_arm"] && config["leader_arm"]["servos"])
        {
            auto leader_servos = config["leader_arm"]["servos"];
            for (size_t i = 0; i < std::min(leader_calibration.size(), leader_servos.size()); ++i)
            {
                auto servo = leader_servos[i];
                if (servo["min"] && servo["max"])
                {
                    leader_calibration[i].min = servo["min"].as<uint16_t>();
                    leader_calibration[i].max = servo["max"].as<uint16_t>();
                }
            }
        }

        // Apply mirroring changes to actual servos
        if (reader_ptr != nullptr && !mirroring_updates.empty())
        {
            mvwprintw(ncurses_win, 27, 0, "Applying mirroring settings to servos...");
            wrefresh(ncurses_win);

            for (const auto& update : mirroring_updates)
            {
                uint8_t servo_id = update.first;
                bool enable_mirroring = update.second;

                try
                {
                    // Clear the serial port first
                    int fd = reader_ptr->getSerialPort().native_handle();
                    ::tcflush(fd, TCIOFLUSH);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));

                    // Set torque enable/disable based on mirroring
                    reader_ptr->writeControlRegister(servo_id, 0x28, enable_mirroring ? 1 : 0);
                    mvwprintw(ncurses_win, 27, 0, "Updated servo %d: mirroring %s",
                              servo_id, enable_mirroring ? "ON" : "OFF");
                    wrefresh(ncurses_win);

                    // Add sufficient delay between commands
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                catch (const std::exception& e)
                {
                    mvwprintw(ncurses_win, 27, 0, "Error setting servo %d: %s", servo_id, e.what());
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }

            mvwprintw(ncurses_win, 27, 0, "                                                    ");
        }

        mvwprintw(ncurses_win, 26, 0, "Calibration loaded successfully!");
        wrefresh(ncurses_win);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

std::ofstream g_debug_log("follower_debug.log");

void testServoControl(ST3215ServoReader& reader)
{
    std::ofstream test_log("servo_test.log");
    test_log << "Starting servo test..." << std::endl;

    // First, try to ping all servos to ensure basic communication
    test_log << "Performing basic communication test..." << std::endl;
    std::vector<uint8_t> responsive_servos;

    for (uint8_t servo_id = 1; servo_id <= 6; servo_id++)
    {
        try
        {
            // Try to read current position - just to check if servo is responsive
            uint16_t current_pos = reader.readPosition(servo_id);
            test_log << "Servo " << (int)servo_id << " is responsive. Current position: " << current_pos << std::endl;
            responsive_servos.push_back(servo_id);
        }
        catch (const std::exception& e)
        {
            test_log << "Servo " << (int)servo_id << " not responding: " << e.what() << std::endl;
        }
    }

    test_log << "Found " << responsive_servos.size() << " responsive servos." << std::endl;

    // Try to move each responsive servo to a specific position
    if (!responsive_servos.empty())
    {
        test_log << "Testing servo control..." << std::endl;

        for (uint8_t servo_id : responsive_servos)
        {
            try
            {
                // First enable torque
                test_log << "Enabling torque for servo " << (int)servo_id << std::endl;

                // Try multiple times to enable torque
                const int MAX_ATTEMPTS = 3;
                bool torque_enabled = false;

                for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++)
                {
                    try
                    {
                        reader.writeControlRegister(servo_id, 0x28, 1);  // Enable torque
                        torque_enabled = true;
                        test_log << "Successfully enabled torque for servo " << (int)servo_id << std::endl;
                        break;
                    }
                    catch (const std::exception& e)
                    {
                        test_log << "Attempt " << (attempt + 1) << " failed to enable torque: " << e.what() << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }

                if (!torque_enabled)
                {
                    test_log << "Could not enable torque for servo " << (int)servo_id << " after " << MAX_ATTEMPTS << " attempts" << std::endl;
                    continue;
                }

                // Read current position
                uint16_t current_pos = reader.readPosition(servo_id);
                test_log << "Current position of servo " << (int)servo_id << ": " << current_pos << std::endl;

                // Move to middle position (around 2048)
                uint16_t target_pos = 2048;
                test_log << "Moving servo " << (int)servo_id << " to position " << target_pos << std::endl;

                int success_count = 0;
                writeServoPositionWithRetry(reader, servo_id, target_pos, success_count);

                // Wait a bit longer for movement to complete
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // Read new position
                uint16_t new_pos = reader.readPosition(servo_id);
                test_log << "New position of servo " << (int)servo_id << ": " << new_pos << std::endl;

                // Calculate difference
                int diff = abs(static_cast<int>(new_pos) - static_cast<int>(target_pos));
                if (diff < 200)
                {
                    test_log << "Servo " << (int)servo_id << " moved successfully (diff: " << diff << ")" << std::endl;
                }
                else
                {
                    test_log << "Servo " << (int)servo_id << " did not move as expected (diff: " << diff << ")" << std::endl;
                }

                // Disable torque
                reader.writeControlRegister(servo_id, 0x28, 0);  // Disable torque
                test_log << "Disabled torque for servo " << (int)servo_id << std::endl;
            }
            catch (const std::exception& e)
            {
                test_log << "ERROR testing servo " << (int)servo_id << ": " << e.what() << std::endl;

                // Try to disable torque in case of error
                try
                {
                    reader.writeControlRegister(servo_id, 0x28, 0);
                }
                catch (...)
                {
                    // Ignore errors when disabling
                }
            }

            // Add delay between servos to avoid bus contention
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    test_log << "Servo test complete" << std::endl;
    test_log.close();
}
// This is a robust function to handle servo positioning with better error recovery
void writeServoPositionWithRetry(ST3215ServoReader& reader, uint8_t servo_id, uint16_t position, int& success_count)
{
    const int MAX_ATTEMPTS = 3;
    const int RETRY_DELAY_MS = 50;

    for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++)
    {
        try
        {
            // First check if we can successfully read from the servo to ensure connection is good
            if (attempt > 0)
            {
                // Only do this check on retry attempts
                try
                {
                    // Just try to read the current position to verify communication
                    uint16_t current = reader.readPosition(servo_id);
                    g_debug_log << "  Retry " << attempt << ": Successfully read position " << current
                                << " from servo " << (int)servo_id << std::endl;
                }
                catch (const std::exception& e)
                {
                    g_debug_log << "  Retry " << attempt << ": Cannot read from servo "
                                << (int)servo_id << ": " << e.what() << std::endl;
                    // Continue with write attempt anyway
                }

                // Flush port before retry
                int fd = reader._serial_port.native_handle();
                ::tcflush(fd, TCIOFLUSH);
                std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS));
            }

            // Try to write the position
            g_debug_log << "  Attempt " << attempt + 1 << ": Writing position " << position
                        << " to servo " << (int)servo_id << std::endl;
            reader.writePosition(servo_id, position);

            // If no exception was thrown, increment success count and return
            success_count++;
            g_debug_log << "  Successfully wrote position to servo " << (int)servo_id << std::endl;
            return;
        }
        catch (const std::exception& e)
        {
            g_debug_log << "  Error on attempt " << attempt + 1 << " writing to servo "
                        << (int)servo_id << ": " << e.what() << std::endl;

            if (attempt == MAX_ATTEMPTS - 1)
            {
                // Last attempt failed, rethrow
                throw;
            }

            // Wait before retry, with increasing delay
            std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS * (attempt + 1)));
        }
    }
}

// Updated function for the ServoPositionsCallback
void robustPositionHandler(const perseus::ServoPositionsMessage& message,
                           std::vector<ServoData>& arm_data,
                           ST3215ServoReader& reader)
{
    g_debug_log << "Received position message with " << sizeof(message.servos) / sizeof(message.servos[0])
                << " servo positions" << std::endl;

    int successful_writes = 0;
    std::vector<uint8_t> failed_servos;

    // Process each servo in sequence
    for (int i = 0; i < 6; i++)
    {
        g_debug_log << "Processing servo " << i + 1
                    << ", mirroring=" << (arm_data[i].mirroring ? "true" : "false")
                    << ", leader_pos=" << message.servos[i].position
                    << ", leader_min=" << leader_calibration[i].min
                    << ", leader_max=" << leader_calibration[i].max
                    << ", follower_min=" << arm_data[i].min
                    << ", follower_max=" << arm_data[i].max << std::endl;

        try
        {
            if (arm_data[i].mirroring)
            {
                // Get leader position and range
                uint16_t leader_position = message.servos[i].position;
                uint16_t leader_min = leader_calibration[i].min;
                uint16_t leader_max = leader_calibration[i].max;

                // Get follower range
                uint16_t follower_min = arm_data[i].min;
                uint16_t follower_max = arm_data[i].max;

                // Calculate follower position
                uint16_t follower_position;

                // Prevent division by zero and handle edge cases
                if (leader_max == leader_min)
                {
                    // If leader has no range, use middle of follower range
                    follower_position = (follower_min + follower_max) / 2;
                    g_debug_log << "  Using mid-point due to equal min/max: " << follower_position << std::endl;
                }
                else
                {
                    // Calculate normalized position (0.0 to 1.0)
                    double normalized = static_cast<double>(leader_position - leader_min) /
                                        static_cast<double>(leader_max - leader_min);

                    // Clamp normalized position between 0 and 1
                    normalized = std::max(0.0, std::min(1.0, normalized));

                    // Scale to follower range
                    follower_position = follower_min + static_cast<uint16_t>(normalized *
                                                                             (follower_max - follower_min));

                    g_debug_log << "  Normalized: " << normalized
                                << ", Calculated position: " << follower_position << std::endl;
                }

                // Update our data structure
                arm_data[i].current = follower_position;
                arm_data[i].torque = message.servos[i].torque;

                // Send to physical servo with retry logic
                writeServoPositionWithRetry(reader, i + 1, follower_position, successful_writes);
            }
        }
        catch (const std::exception& e)
        {
            g_debug_log << "  ERROR: " << e.what() << std::endl;
            arm_data[i].error = std::string("Mirror error: ") + e.what();
            failed_servos.push_back(i + 1);
        }
    }

    g_debug_log << "Position message processing complete: " << successful_writes
                << " successful writes, " << failed_servos.size() << " failures" << std::endl;

    if (!failed_servos.empty())
    {
        g_debug_log << "Failed servos: ";
        for (auto id : failed_servos)
        {
            g_debug_log << (int)id << " ";
        }
        g_debug_log << std::endl;
    }

    g_debug_log.flush();
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

        // Test servo control to verify hardware is working
        // Test servo control to verify hardware is working
        testServoControl(*reader_ptr);

        // Pre-enable mirroring on servos with proper range values
        for (int i = 0; i < arm_data.size(); i++)
        {
            // Set reasonable default min/max values that span the servo's range
            arm_data[i].min = 1000;  // Lower than center
            arm_data[i].max = 3000;  // Higher than center
            arm_data[i].mirroring = true;

            try
            {
                g_debug_log << "Enabling mirroring for servo " << (i + 1) << " at startup" << std::endl;

                // Enable torque for the servo
                reader_ptr->writeControlRegister(i + 1, 0x28, 1);
                g_debug_log << "Successfully enabled torque for servo " << (i + 1) << std::endl;
            }
            catch (const std::exception& e)
            {
                g_debug_log << "Error enabling mirroring for servo " << (i + 1) << ": " << e.what() << std::endl;
                arm_data[i].error = std::string("Startup error: ") + e.what();
                // Don't disable mirroring here
            }
        }

        // Set up network interface for receiving commands
        uint16_t listen_port = (argc > 2) ? static_cast<uint16_t>(std::stoi(argv[2])) : perseus::DEFAULT_PORT;
        network_ptr = new perseus::ArmNetworkInterface(listen_port);

        // Set up callbacks for network messages
        // Modified callback function for the follower-main.cpp file
        // Fixed position mapping function for follower-main.cpp
        network_ptr->setServoPositionsCallback([&arm_data, &reader](const perseus::ServoPositionsMessage& message)
                                               { robustPositionHandler(message, arm_data, reader); });

        network_ptr->setServoMirroringCallback([&arm_data, &reader](const perseus::ServoMirroringMessage& message)
                                               {
                g_debug_log << "Received mirroring message for servo " << (int)message.servo_id 
                            << ", mirroring=" << (message.mirroring ? "true" : "false") << std::endl;
                
                if (message.servo_id >= 1 && message.servo_id <= 6) {
                    size_t idx = message.servo_id - 1;
                    arm_data[idx].mirroring = message.mirroring;
                    
                    try {
                        // Enable/disable torque based on mirroring state
                        g_debug_log << "  Setting torque " << (message.mirroring ? "ON" : "OFF") 
                                    << " for servo " << (int)message.servo_id << std::endl;
                        reader.writeControlRegister(message.servo_id, 0x28, message.mirroring ? 1 : 0);
                        g_debug_log << "  Successfully set torque" << std::endl;
                        
                        if (!message.mirroring) {
                            arm_data[idx].error.clear();
                        }
                    } catch (const std::exception& e) {
                        g_debug_log << "  ERROR setting torque: " << e.what() << std::endl;
                        arm_data[idx].error = std::string("Torque control error: ") + e.what();
                    }
                }
                g_debug_log.flush(); });

        network_ptr->setCalibrationCallback([&arm_data](const perseus::CalibrationMessage& message)
                                            {
                    g_debug_log << "Received calibration message" << std::endl;
                    
                    for (int i = 0; i < 6; i++) {
                        // Store leader's calibration values for mapping
                        leader_calibration[i].min = message.servos[i].min;
                        leader_calibration[i].max = message.servos[i].max;
                        
                        g_debug_log << "  Servo " << i+1 << " leader calibration: min=" 
                                    << leader_calibration[i].min << ", max=" << leader_calibration[i].max << std::endl;
                    }
                    g_debug_log.flush(); });

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

                    // Always update min/max based on actual positions (not just in calibration mode)
                    updateServoCalibration(arm_data, current_position, i);

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
                    arm_data[i].error = e.what();
                    g_debug_log << "Error with servo " << (i + 1) << ": " << e.what() << std::endl;

                    // Don't disable mirroring - just log the error and continue

                    // Only reset torque value for display purposes
                    arm_data[i].torque = 0;  // Reset torque on error

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
            }

            // Update display with arm data and network status
            displayServoValues(ncurses_win,
                               arm_data,
                               leader_calibration,  // This parameter was missing
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
            else if (ch == 's' || ch == 'S')
            {
                mvwprintw(ncurses_win, 26, 0, "Saving calibration data...");
                wrefresh(ncurses_win);

                try
                {
                    exportCalibrationData(arm_data, leader_calibration, port_path);
                    mvwprintw(ncurses_win, 26, 0, "                                                                        ");
                    mvwprintw(ncurses_win, 26, 0, "Calibration data saved successfully! Press any key to continue");
                    wrefresh(ncurses_win);

                    // Wait for any key
                    nodelay(ncurses_win, FALSE);  // Switch to blocking mode temporarily
                    wgetch(ncurses_win);
                    nodelay(ncurses_win, TRUE);  // Switch back to non-blocking

                    // Clear status line
                    mvwprintw(ncurses_win, 26, 0, "                                                                        ");
                    wrefresh(ncurses_win);
                }
                catch (const std::exception& e)
                {
                    mvwprintw(ncurses_win, 26, 0, "                                                    ");
                    mvwprintw(ncurses_win, 26, 0, "Error saving calibration: %s", e.what());
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }

                mvwprintw(ncurses_win, 26, 0, "                                                    ");
                wrefresh(ncurses_win);
            }
            else if (ch == 'l' || ch == 'L')
            {
                mvwprintw(ncurses_win, 26, 0, "Loading calibration data...");
                wrefresh(ncurses_win);

                try
                {
                    loadCalibrationData(arm_data, leader_calibration);
                    mvwprintw(ncurses_win, 26, 0, "                                                    ");
                    mvwprintw(ncurses_win, 26, 0, "Calibration data loaded successfully!");
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                catch (const std::exception& e)
                {
                    mvwprintw(ncurses_win, 26, 0, "                                                    ");
                    mvwprintw(ncurses_win, 26, 0, "Error loading calibration: %s", e.what());
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }

                mvwprintw(ncurses_win, 26, 0, "                                                    ");
                wrefresh(ncurses_win);
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
