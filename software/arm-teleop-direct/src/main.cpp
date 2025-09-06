// Include guard to handle Boost.Asio before ncurses
#ifndef PERSEUS_ARM_ASIO_INCLUDES
#define PERSEUS_ARM_ASIO_INCLUDES
#include "perseus-arm-teleop.hpp"
#endif

#include <curses.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
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

#include "servo-constants.hpp"

// Use the servo constants namespace for easier access to constants
using namespace perseus::servo;

// Global variables for cleanup
static ST3215ServoReader* reader1_ptr = nullptr;
static ST3215ServoReader* reader2_ptr = nullptr;
static WINDOW* ncurses_win = nullptr;

static std::atomic<bool> running(true);
static std::atomic<bool> torque_protection(false);  // Global flag for torque protection

// Find available serial ports
std::vector<std::string> findSerialPorts()
{
    std::vector<std::string> ports;
    const std::filesystem::path dev_path("/dev");
    const std::filesystem::path virtual_port_symlink("/home/dingo/leader_follower");

    // Check if the symlink to the virtual port exists and add it
    if (std::filesystem::exists(virtual_port_symlink))
    {
        // Add the symlink path as-is, since we want to use the symlink itself
        ports.push_back(virtual_port_symlink.string());
    }

    // Find physical serial ports
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
// Let user select ports for both arms
std::pair<std::string, std::string> selectSerialPorts(const std::vector<std::string>& ports)
{
    if (ports.empty())
    {
        throw std::runtime_error("No serial ports found");
    }

    std::cout << "\nSelect ports for Perseus arms control\n";
    std::cout << "=====================================\n";
    std::cout << "Available serial ports:\n";
    for (size_t i = 0; i < ports.size(); ++i)
    {
        std::cout << i + 1 << ": " << ports[i] << std::endl;
    }
    std::cout << "0: Rescan for ports\n\n";

    std::string port1, port2;
    size_t selection1 = 0, selection2 = 0;

    // Select first arm port
    while (true)
    {
        std::cout << "Select port for ARM 1 (0 to rescan, 1-" << ports.size() << " to select): ";
        if (!(std::cin >> selection1))
        {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (selection1 == 0)
        {
            return selectSerialPorts(findSerialPorts());  // Rescan and restart
        }

        if (selection1 > 0 && selection1 <= ports.size())
        {
            port1 = ports[selection1 - 1];
            break;
        }

        std::cout << "Invalid selection. Please try again.\n";
    }

    // Display confirmation and proceed to second arm
    std::cout << "\nARM 1 will use: " << port1 << "\n\n";

    // Select second arm port
    while (true)
    {
        std::cout << "Select port for ARM 2 (0 to rescan, 1-" << ports.size() << " to select): ";
        if (!(std::cin >> selection2))
        {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (selection2 == 0)
        {
            return selectSerialPorts(findSerialPorts());  // Rescan and restart
        }

        if (selection2 > 0 && selection2 <= ports.size() && selection2 != selection1)
        {
            port2 = ports[selection2 - 1];
            break;
        }

        if (selection2 == selection1)
        {
            std::cout << "Cannot use the same port for both arms. Please select a different port.\n";
        }
        else
        {
            std::cout << "Invalid selection. Please try again.\n";
        }
    }

    std::cout << "\nARM 2 will use: " << port2 << "\n";
    std::cout << "\nPress Enter to continue...";
    std::cin.ignore(10000, '\n');
    std::cin.get();

    return {port1, port2};
}

// Create a colored progress bar string
void displayProgressBar(WINDOW* ncurses_win, int y, int x, uint16_t current, uint16_t min, uint16_t max)
{
    // Clamp values to 0-4095
    current = std::min(current, static_cast<uint16_t>(limits::MAX_POSITION));
    min = std::min(min, static_cast<uint16_t>(limits::MAX_POSITION));
    max = std::min(max, static_cast<uint16_t>(limits::MAX_POSITION));

    const size_t barLength = ui::POSITION_BAR_LENGTH;

    // Calculate positions
    size_t currentPos = static_cast<size_t>((static_cast<double>(current) / limits::MAX_POSITION) * barLength);
    size_t minPos = static_cast<size_t>((static_cast<double>(min) / limits::MAX_POSITION) * barLength);
    size_t maxPos = static_cast<size_t>((static_cast<double>(max) / limits::MAX_POSITION) * barLength);

    // Print opening bracket
    mvwaddch(ncurses_win, y, x, '[');
    x++;

    // Print bar with colors
    for (size_t i = 0; i < barLength; i++)
    {
        if (has_colors())
        {
            if (i == minPos)
            {
                wattron(ncurses_win, COLOR_PAIR(1));  // Blue for min
                waddch(ncurses_win, '#');
                wattroff(ncurses_win, COLOR_PAIR(1));
            }
            else if (i == maxPos)
            {
                wattron(ncurses_win, COLOR_PAIR(2));  // Green for max
                waddch(ncurses_win, '#');
                wattroff(ncurses_win, COLOR_PAIR(2));
            }
            else if (i < currentPos)
            {
                if (i < minPos)
                {
                    wattron(ncurses_win, COLOR_PAIR(3) | A_DIM);  // Dimmed white for positions before min
                    waddch(ncurses_win, '#');
                    wattroff(ncurses_win, COLOR_PAIR(3) | A_DIM);
                }
                else
                {
                    wattron(ncurses_win, COLOR_PAIR(3));  // Bright white for current valid position
                    waddch(ncurses_win, '#');
                    wattroff(ncurses_win, COLOR_PAIR(3));
                }
            }
            else
            {
                waddch(ncurses_win, ' ');
            }
        }
        else
        {
            // For non-color displays, still show all positions but with different characters
            if (i < currentPos)
            {
                waddch(ncurses_win, (i < minPos) ? '.' : '#');
            }
            else
            {
                waddch(ncurses_win, ' ');
            }
        }
    }

    // Print closing bracket
    waddch(ncurses_win, ']');
}

// Structure to hold servo data including min/max values
struct ServoData
{
    uint16_t current = 0;
    uint16_t min = limits::MAX_POSITION;
    uint16_t max = 0;
    int16_t torque = 0;  // New field for torque feedback
    std::string error;
    bool mirroring = false;  // Whether this servo is mirroring arm 1
};

// Create a colored torque bar string
void displayTorqueBar(WINDOW* win, int y, int x, int16_t torque)
{
    const int width = ui::TORQUE_BAR_WIDTH;
    const int16_t max_display = ui::MAX_DISPLAY_TORQUE;  // Scale display to ±100

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

std::string getWorkingDirectory()
{
    return std::filesystem::current_path().string();
}

// Display servo values in ncurses window for both arms
// Display servo values in ncurses window for both arms
void displayServoValues([[maybe_unused]] WINDOW* win,
                        const std::vector<ServoData>& arm1_data,
                        const std::vector<ServoData>& arm2_data)
{
    werase(win);

    // Display header
    mvwprintw(win, 0, 0, "Perseus Arms Servo Positions (0-4095) and Torque (-100 to +100)");
    mvwprintw(win, 1, 0, "--------------------------------------------------------");

    // Column headers
    mvwprintw(win, 2, 2, "Servo    Current    Min      Max      Torque  Range");
    mvwprintw(win, 3, 0, "--------------------------------------------------------");

    // Display first arm's servos
    mvwprintw(win, 4, 0, "Arm 1:");
    for (size_t i = 0; i < 6; ++i)
    {
        int row = i + 5;
        const auto& servo = arm1_data[i];

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

    mvwprintw(ncurses_win, 11, 0, "--------------------------------------------------------");

    // Display second arm's servos
    mvwprintw(ncurses_win, 12, 0, "Arm 2:");
    for (size_t i = 0; i < 6; ++i)
    {
        int row = i + 13;
        const auto& servo = arm2_data[i];

        if (servo.error.empty())
        {
            if (servo.mirroring && has_colors())
            {
                wattron(ncurses_win, COLOR_PAIR(4) | A_BOLD);
            }

            mvwprintw(ncurses_win, row, 2, "%-8d %8u  %8u  %8u  [Torque: %5d] ",
                      static_cast<int>(i + 1),
                      servo.current,
                      servo.min,
                      servo.max,
                      servo.torque);

            // Display torque bar
            displayTorqueBar(ncurses_win, row, 42, servo.torque);

            // Display position bar with adjusted x position to make room for torque
            displayProgressBar(ncurses_win, row, 49, servo.current, servo.min, servo.max);

            if (servo.mirroring && has_colors())
            {
                wattroff(ncurses_win, COLOR_PAIR(4) | A_BOLD);
            }
        }
        else
        {
            mvwprintw(ncurses_win, row, 2, "%-8d Error: %s",
                      static_cast<int>(i + 1),
                      servo.error.c_str());
        }
    }

    mvwprintw(ncurses_win, 19, 0, "--------------------------------------------------------");

    // Add instructions and working directory
    mvwprintw(ncurses_win, 20, 0, "Instructions:");
    mvwprintw(ncurses_win, 21, 0, "1. Move both arms through their full range of motion");
    mvwprintw(ncurses_win, 22, 0, "2. Press 's' to save calibration when done");
    mvwprintw(ncurses_win, 23, 0, "3. Press keys 1-6 to toggle mirroring for each servo");
    mvwprintw(ncurses_win, 24, 0, "4. Press 't' to toggle torque protection (%s)",
              torque_protection.load() ? "ON" : "OFF");
    mvwprintw(ncurses_win, 25, 0, "5. Press 'l' to load calibration from file");
    mvwprintw(ncurses_win, 26, 0, "6. Press Ctrl+C to exit");

    // Add color legend if colors are available
    if (has_colors())
    {
        mvwprintw(ncurses_win, 27, 0, "Legend: ");
        wattron(ncurses_win, COLOR_PAIR(4) | A_BOLD);
        wprintw(ncurses_win, "Yellow rows = Mirrored servos");
        wattroff(ncurses_win, COLOR_PAIR(4) | A_BOLD);
    }
    mvwprintw(ncurses_win, 28, 0, "Save directory: %s", getWorkingDirectory().c_str());

    wrefresh(ncurses_win);
}

void exportCalibrationData(const std::vector<ServoData>& arm1_data,
                           const std::vector<ServoData>& arm2_data,
                           const std::string& port1,
                           const std::string& port2)
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
        config["arm1_port"] = port1;
        config["arm2_port"] = port2;

        // Add calibration data for arm 1
        YAML::Node arm1_node;
        for (size_t i = 0; i < arm1_data.size(); ++i)
        {
            YAML::Node servo;
            servo["id"] = i + 1;
            servo["min"] = arm1_data[i].min;
            servo["max"] = arm1_data[i].max;
            // Add current mirroring state
            servo["mirroring"] = false;  // Arm 1 servos are never mirrored
            arm1_node["servos"].push_back(servo);
        }
        config["arm1"] = arm1_node;

        // Add calibration data for arm 2
        YAML::Node arm2_node;
        for (size_t i = 0; i < arm2_data.size(); ++i)
        {
            YAML::Node servo;
            servo["id"] = i + 1;
            servo["min"] = arm2_data[i].min;
            servo["max"] = arm2_data[i].max;
            // Include mirroring state for arm 2
            servo["mirroring"] = arm2_data[i].mirroring;
            arm2_node["servos"].push_back(servo);
        }
        config["arm2"] = arm2_node;

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

void loadCalibrationData(std::vector<ServoData>& arm1_data,
                         std::vector<ServoData>& arm2_data,
                         const std::string& filename = "arm_calibration.yaml")
{
    try
    {
        YAML::Node config = YAML::LoadFile(filename);

        // Load arm 1 servo data
        if (config["arm1"] && config["arm1"]["servos"])
        {
            auto arm1_servos = config["arm1"]["servos"];
            for (size_t i = 0; i < std::min(arm1_data.size(), arm1_servos.size()); ++i)
            {
                auto servo = arm1_servos[i];
                if (servo["min"] && servo["max"])
                {
                    arm1_data[i].min = servo["min"].as<uint16_t>();
                    arm1_data[i].max = servo["max"].as<uint16_t>();
                    // Ensure current position stays within new bounds
                    arm1_data[i].current = std::max(arm1_data[i].min,
                                                    std::min(arm1_data[i].max,
                                                             arm1_data[i].current));
                }
            }
        }

        // Load arm 2 servo data
        if (config["arm2"] && config["arm2"]["servos"])
        {
            auto arm2_servos = config["arm2"]["servos"];
            for (size_t i = 0; i < std::min(arm2_data.size(), arm2_servos.size()); ++i)
            {
                auto servo = arm2_servos[i];
                if (servo["min"] && servo["max"])
                {
                    arm2_data[i].min = servo["min"].as<uint16_t>();
                    arm2_data[i].max = servo["max"].as<uint16_t>();
                    // Ensure current position stays within new bounds
                    arm2_data[i].current = std::max(arm2_data[i].min,
                                                    std::min(arm2_data[i].max,
                                                             arm2_data[i].current));
                }
                if (servo["mirroring"])
                {
                    arm2_data[i].mirroring = servo["mirroring"].as<bool>();
                }
            }
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

void disableTorqueAndCleanup()
{
    // First disable ncurses if it's active
    if (ncurses_win != nullptr)
    {
        mvwprintw(ncurses_win, 27, 0, "Disabling servo torque...");
        wrefresh(ncurses_win);
    }

    // Disable torque for all servos on both arms
    if (reader1_ptr != nullptr)
    {
        for (uint8_t i = 1; i <= 6; ++i)
        {
            try
            {
                reader1_ptr->writeControlRegister(i, register_addr::TORQUE_ENABLE, 0);
            }
            catch (...)
            {
                // Ignore errors during shutdown
            }
        }
    }

    if (reader2_ptr != nullptr)
    {
        for (uint8_t i = 1; i <= 6; ++i)
        {
            try
            {
                reader2_ptr->writeControlRegister(i, register_addr::TORQUE_ENABLE, 0);
            }
            catch (...)
            {
                // Ignore errors during shutdown
            }
        }
    }

    // Clean up ncurses
    if (ncurses_win != nullptr)
    {
        endwin();
        ncurses_win = nullptr;
    }
}

void signalHandler([[maybe_unused]] int signum)
{
    running = false;
    disableTorqueAndCleanup();
}

int main(int argc, char* argv[])
{
    std::vector<ServoData> arm1_data(6);
    std::vector<ServoData> arm2_data(6);
    try
    {
        // Set up signal handling
        signal(SIGINT, signalHandler);

        // Get port paths
        std::string port_path1, port_path2;
        if (argc > 2)
        {
            port_path1 = argv[1];
            port_path2 = argv[2];
        }
        else
        {
            auto available_ports = findSerialPorts();
            auto [p1, p2] = selectSerialPorts(available_ports);
            port_path1 = p1;
            port_path2 = p2;
        }

        std::cout << "Using serial ports:\nArm 1: " << port_path1
                  << "\nArm 2: " << port_path2 << std::endl;
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
        }

        // Initialize servo readers and data storage for both arms
        ST3215ServoReader reader1(port_path1, communication::DEFAULT_BAUD_RATE, communication::DEFAULT_ACCELERATION);
        ST3215ServoReader reader2(port_path2, communication::DEFAULT_BAUD_RATE, communication::DEFAULT_ACCELERATION);
        reader1_ptr = &reader1;
        reader2_ptr = &reader2;

        // Initialize colors for torque display
        if (has_colors())
        {
            start_color();
            init_pair(1, COLOR_BLUE, COLOR_BLACK);    // For min value
            init_pair(2, COLOR_GREEN, COLOR_BLACK);   // For max value
            init_pair(3, COLOR_WHITE, COLOR_BLACK);   // For current value
            init_pair(4, COLOR_YELLOW, COLOR_BLACK);  // For medium torque
            init_pair(5, COLOR_RED, COLOR_BLACK);     // For high torque
            init_pair(6, COLOR_WHITE, COLOR_BLACK);   // For low torque (grey)
        }

        // Main loop
        while (running)
        {
            // Read all servo positions and torque from first arm
            for (uint8_t i = 0; i < 6; ++i)
            {
                try
                {
                    auto& servo = arm1_data[i];
                    servo.current = reader1.readPosition(i + 1);
                    servo.min = std::min(servo.min, servo.current);
                    servo.max = std::max(servo.max, servo.current);

                    // Read torque value
                    int16_t torque = reader1.readLoad(i + 1);
                    servo.torque = torque;

                    // Check if torque exceeds safety threshold
                    if (torque_protection.load() && std::abs(torque) > limits::TORQUE_SAFETY_THRESHOLD)
                    {
                        // Disable torque
                        reader1.writeControlRegister(i + 1, register_addr::TORQUE_ENABLE, 0);
                        servo.error = "Torque limit exceeded - disabled";
                    }
                    else if (!servo.error.empty())
                    {
                        servo.error.clear();
                    }
                }
                catch (const std::exception& e)
                {
                    arm1_data[i].error = e.what();
                    arm1_data[i].torque = 0;  // Clear torque on error
                }
            }

            // Read all servo positions and torque from second arm
            for (uint8_t i = 0; i < 6; ++i)
            {
                try
                {
                    auto& servo = arm2_data[i];
                    servo.current = reader2.readPosition(i + 1);
                    servo.min = std::min(servo.min, servo.current);
                    servo.max = std::max(servo.max, servo.current);

                    // Read torque value if servo is enabled (mirroring)
                    if (servo.mirroring)
                    {
                        int16_t torque = reader2.readLoad(i + 1);
                        servo.torque = torque;

                        // Only apply software torque protection if enabled
                        if (torque_protection.load() && std::abs(torque) > limits::TORQUE_SAFETY_THRESHOLD)
                        {
                            // Disable torque
                            reader2.writeControlRegister(i + 1, register_addr::TORQUE_ENABLE, 0);
                            servo.error = "Torque limit exceeded - disabled";
                            servo.mirroring = false;  // Disable mirroring when torque limit is exceeded
                        }
                        else if (servo.error.find("Overload") != std::string::npos)
                        {
                            // If we have an overload error but torque protection is off, try to re-enable
                            try
                            {
                                reader2.writeControlRegister(i + 1, register_addr::TORQUE_ENABLE, 1);  // Re-enable torque
                                servo.error.clear();
                            }
                            catch (const std::exception& e)
                            {
                                // If re-enable fails, keep the error
                                servo.error = std::string("Failed to re-enable servo: ") + e.what();
                            }
                        }
                        else
                        {
                            servo.error.clear();  // Clear any previous errors
                        }
                    }
                    else
                    {
                        servo.torque = 0;  // Zero torque when servo is not mirroring
                    }

                    if (!servo.error.empty() && servo.mirroring)
                    {
                        servo.error.clear();
                    }
                }
                catch (const std::exception& e)
                {
                    arm2_data[i].error = e.what();
                    arm2_data[i].torque = 0;  // Clear torque on error
                    if (arm2_data[i].mirroring)
                    {
                        arm2_data[i].mirroring = false;  // Disable mirroring on error
                    }
                }
            }

            // Delay to prevent overwhelming servos
            std::this_thread::sleep_for(std::chrono::milliseconds(ui::SERVO_REFRESH_DELAY_MS));
        }

        // ... end of function ...
    }
    catch (const std::exception& e)
    {
        disableTorqueAndCleanup();
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}