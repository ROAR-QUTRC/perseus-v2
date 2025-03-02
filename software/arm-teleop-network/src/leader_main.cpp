#include <curses.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "arm_teleop/leader_controller.hpp"
#include "perseus-arm-teleop.hpp"

const int16_t SERVO_REFRESH_DELAY_MS = 25;
const int16_t TORQUE_SAFETY_THRESHOLD = 800;  // 80% of maximum torque

// Global variables for cleanup
static ST3215ServoReader* reader1_ptr = nullptr;
static WINDOW* ncurses_win = nullptr;
static arm_teleop::LeaderController* leader_controller_ptr = nullptr;

static std::atomic<bool> running(true);
static std::atomic<bool> torque_protection(false);  // Global flag for torque protection

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
    std::cout << "=========================================\n";
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
void displayProgressBar(WINDOW* ncurses_win, int y, int x, uint16_t current, uint16_t min, uint16_t max)
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
typedef arm_teleop::NetworkedArmController::ServoData ServoData;

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

std::string getWorkingDirectory()
{
    return std::filesystem::current_path().string();
}

// Display servo values in ncurses window for leader arm
void displayServoValues(WINDOW* win, const std::vector<ServoData>& arm1_data)
{
    werase(win);

    // Display header
    mvwprintw(win, 0, 0, "Perseus Leader Arm Servo Positions (0-4095) and Torque (-100 to +100)");
    mvwprintw(win, 1, 0, "--------------------------------------------------------");

    // Column headers
    mvwprintw(win, 2, 2, "Servo    Current    Min      Max      Torque  Range");
    mvwprintw(win, 3, 0, "--------------------------------------------------------");

    // Display leader arm's servos
    mvwprintw(win, 4, 0, "Leader Arm:");
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

    mvwprintw(win, 11, 0, "--------------------------------------------------------");

    // Show network status
    if (leader_controller_ptr)
    {
        mvwprintw(win, 12, 0, "Network Status: %s",
                  leader_controller_ptr->getStatusMessage().c_str());
    }
    else
    {
        mvwprintw(win, 12, 0, "Network Status: Not initialized");
    }

    mvwprintw(win, 13, 0, "--------------------------------------------------------");

    // Add instructions
    mvwprintw(win, 14, 0, "Instructions:");
    mvwprintw(win, 15, 0, "1. Move arm through its full range of motion");
    mvwprintw(win, 16, 0, "2. Press 's' to save calibration and update follower");
    mvwprintw(win, 17, 0, "3. Press keys 1-6 to toggle mirroring for each servo");
    mvwprintw(win, 18, 0, "4. Press 't' to toggle torque protection (%s)",
              torque_protection.load() ? "ON" : "OFF");
    mvwprintw(win, 19, 0, "5. Press 'l' to load calibration from file");
    mvwprintw(win, 20, 0, "6. Press Ctrl+C to exit");

    mvwprintw(win, 21, 0, "Save directory: %s", getWorkingDirectory().c_str());

    wrefresh(win);
}

void exportCalibrationData(const std::vector<ServoData>& arm1_data,
                           const std::string& port1)
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

        // Add calibration data for arm 1
        YAML::Node arm1_node;
        for (size_t i = 0; i < arm1_data.size(); ++i)
        {
            YAML::Node servo;
            servo["id"] = i + 1;
            servo["min"] = arm1_data[i].min;
            servo["max"] = arm1_data[i].max;
            arm1_node["servos"].push_back(servo);
        }
        config["arm1"] = arm1_node;

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
        mvwprintw(ncurses_win, 22, 0, "Disabling servo torque and cleaning up...");
        wrefresh(ncurses_win);
    }

    // Stop network controller
    if (leader_controller_ptr != nullptr)
    {
        leader_controller_ptr->stop();
    }

    // Disable torque for all servos on the arm
    if (reader1_ptr != nullptr)
    {
        for (uint8_t i = 1; i <= 6; ++i)
        {
            try
            {
                reader1_ptr->writeControlRegister(i, 0x28, 0);  // 0x28 is torque enable register
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

void showNetworkConfiguration(const std::string& server_address, int port)
{
    std::cout << "\nNetwork Configuration:\n";
    std::cout << "=====================\n";
    std::cout << "Connect to server: " << server_address << ":" << port << "\n\n";
    std::cout << "Press Enter to continue...";
    std::cin.ignore(10000, '\n');
    std::string dummy;
    std::getline(std::cin, dummy);
}

int main(int argc, char* argv[])
{
    std::vector<ServoData> arm1_data(6);
    std::string port_path;
    std::string server_address = "localhost";
    int server_port = arm_teleop::protocol::DEFAULT_PORT;

    try
    {
        // Set up signal handling
        signal(SIGINT, signalHandler);

        // Process command-line arguments for network configuration
        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];
            if ((arg == "--server" || arg == "-s") && i + 1 < argc)
            {
                server_address = argv[++i];
            }
            else if ((arg == "--port" || arg == "-p") && i + 1 < argc)
            {
                server_port = std::stoi(argv[++i]);
            }
            else if ((arg == "--help" || arg == "-h"))
            {
                std::cout << "Usage: " << argv[0] << " [options]\n"
                          << "Options:\n"
                          << "  -s, --server ADDRESS  Server address (default: localhost)\n"
                          << "  -p, --port PORT       Server port (default: " << arm_teleop::protocol::DEFAULT_PORT << ")\n"
                          << "  -h, --help            Show this help message\n";
                return 0;
            }
            else if (i == 1 && arg.find("tty") != std::string::npos)
            {
                // Assume first non-option argument is the serial port
                port_path = arg;
            }
        }

        showNetworkConfiguration(server_address, server_port);

        // Get port path if not specified
        if (port_path.empty())
        {
            auto available_ports = findSerialPorts();
            port_path = selectSerialPort(available_ports);
        }

        std::cout << "Using serial port: " << port_path << std::endl;
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
            init_pair(5, COLOR_RED, COLOR_BLACK);     // For high torque
            init_pair(6, COLOR_WHITE, COLOR_BLACK);   // For low torque (grey)
        }

        // Initialize servo reader for leader arm
        ST3215ServoReader reader1(port_path, 1000000, 30);  // 30 sets the acceleration
        reader1_ptr = &reader1;

        // Initialize network controller
        arm_teleop::LeaderController leader_controller(server_address, server_port);
        leader_controller_ptr = &leader_controller;

        // Try to initialize the network connection
        bool network_initialized = leader_controller.initialize();
        if (!network_initialized)
        {
            mvwprintw(ncurses_win, 23, 0, "Warning: Failed to connect to follower. Will retry in background.");
            wrefresh(ncurses_win);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        // Start network thread regardless (it will try to reconnect)
        leader_controller.start();

        // Main loop
        while (running)
        {
            // Read all servo positions and torque from arm
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
                    if (torque_protection.load() && std::abs(torque) > TORQUE_SAFETY_THRESHOLD)
                    {
                        // Disable torque
                        reader1.writeControlRegister(i + 1, 0x28, 0);  // 0x28 is torque enable register
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

            // Update the network controller with servo data
            leader_controller.updateServoData(arm1_data);

            // Update display
            displayServoValues(ncurses_win, arm1_data);

            // Handle keyboard input
            int ch = wgetch(ncurses_win);
            if (ch == 't' || ch == 'T')
            {
                bool new_state = !torque_protection.load();
                torque_protection.store(new_state);

                mvwprintw(ncurses_win, 22, 0, "                                                                        ");
                if (has_colors())
                {
                    wattron(ncurses_win, COLOR_PAIR(new_state ? 2 : 5) | A_BOLD);
                }
                mvwprintw(ncurses_win, 22, 0, "Torque protection %s", new_state ? "ENABLED" : "DISABLED");
                if (has_colors())
                {
                    wattroff(ncurses_win, COLOR_PAIR(new_state ? 2 : 5) | A_BOLD);
                }
                wrefresh(ncurses_win);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                mvwprintw(ncurses_win, 22, 0, "                                                                        ");
            }
            else if (ch == 'l' || ch == 'L')
            {
                mvwprintw(ncurses_win, 23, 0, "Loading calibration data...");
                wrefresh(ncurses_win);

                try
                {
                    loadCalibrationData(arm1_data);
                    mvwprintw(ncurses_win, 23, 0, "                                                    ");
                    mvwprintw(ncurses_win, 23, 0, "Calibration data loaded successfully!");
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                    // Send calibration to follower
                    if (leader_controller.sendCalibrationData(arm1_data))
                    {
                        mvwprintw(ncurses_win, 23, 0, "                                                    ");
                        mvwprintw(ncurses_win, 23, 0, "Calibration data sent to follower successfully!");
                    }
                    else
                    {
                        mvwprintw(ncurses_win, 23, 0, "                                                    ");
                        mvwprintw(ncurses_win, 23, 0, "Failed to send calibration to follower.");
                    }
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                catch (const std::exception& e)
                {
                    mvwprintw(ncurses_win, 23, 0, "                                                    ");
                    mvwprintw(ncurses_win, 23, 0, "Error loading calibration: %s", e.what());
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }

                mvwprintw(ncurses_win, 23, 0, "                                                    ");
                wrefresh(ncurses_win);
            }
            else if (ch == 's' || ch == 'S')
            {
                mvwprintw(ncurses_win, 23, 0, "Saving calibration data...");
                wrefresh(ncurses_win);

                try
                {
                    exportCalibrationData(arm1_data, port_path);

                    // Send calibration to follower
                    bool sent_to_follower = leader_controller.sendCalibrationData(arm1_data);

                    mvwprintw(ncurses_win, 23, 0, "                                                                        ");
                    mvwprintw(ncurses_win, 23, 0, "Calibration data saved successfully! %s",
                              sent_to_follower ? "Sent to follower." : "Failed to send to follower.");
                    mvwprintw(ncurses_win, 24, 0, "Press any key to continue");
                    wrefresh(ncurses_win);

                    // Wait for any key
                    nodelay(ncurses_win, FALSE);  // Switch to blocking mode temporarily
                    wgetch(ncurses_win);
                    nodelay(ncurses_win, TRUE);  // Switch back to non-blocking

                    // Clear status line
                    mvwprintw(ncurses_win, 23, 0, "                                                                        ");
                    mvwprintw(ncurses_win, 24, 0, "                                                                        ");
                    wrefresh(ncurses_win);
                }
                catch (const std::exception& e)
                {
                    mvwprintw(ncurses_win, 23, 0, "                                                                        ");
                    mvwprintw(ncurses_win, 23, 0, "Error saving calibration: %s", e.what());
                    wrefresh(ncurses_win);
                    std::this_thread::sleep_for(std::chrono::seconds(2));

                    // Clear error message
                    mvwprintw(ncurses_win, 23, 0, "                                                                        ");
                    wrefresh(ncurses_win);
                }
            }
            else if (ch >= '1' && ch <= '6')
            {
                int servo_idx = ch - '1';

                // Toggle mirroring state
                bool new_mirror_state = !arm1_data[servo_idx].mirroring;

                // Send command to follower
                bool success = leader_controller.sendMirrorCommand(servo_idx + 1, new_mirror_state);

                // Update local state
                if (success)
                {
                    arm1_data[servo_idx].mirroring = new_mirror_state;

                    // Update display immediately with color
                    mvwprintw(ncurses_win, 23, 0, "                                                                        ");
                    if (has_colors())
                    {
                        wattron(ncurses_win, COLOR_PAIR(4) | A_BOLD);
                    }
                    mvwprintw(ncurses_win, 23, 0, "Servo %d mirroring %s",
                              servo_idx + 1,
                              new_mirror_state ? "enabled" : "disabled");
                    if (has_colors())
                    {
                        wattroff(ncurses_win, COLOR_PAIR(4) | A_BOLD);
                    }
                }
                else
                {
                    // Handle error
                    mvwprintw(ncurses_win, 23, 0, "Failed to set mirroring for servo %d", servo_idx + 1);
                }

                wrefresh(ncurses_win);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                mvwprintw(ncurses_win, 23, 0, "                                                                        ");
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