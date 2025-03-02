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

#include "arm_teleop/follower_controller.hpp"
#include "perseus-arm-teleop.hpp"

const int16_t SERVO_REFRESH_DELAY_MS = 25;
const int16_t TORQUE_SAFETY_THRESHOLD = 800;  // 80% of maximum torque

// Global variables for cleanup
static ST3215ServoReader* reader2_ptr = nullptr;
static WINDOW* ncurses_win = nullptr;
static arm_teleop::FollowerController* follower_controller_ptr = nullptr;

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

// Let user select port for the follower arm
std::string selectSerialPort(const std::vector<std::string>& ports)
{
    if (ports.empty())
    {
        throw std::runtime_error("No serial ports found");
    }

    std::cout << "\nSelect port for Perseus follower arm control\n";
    std::cout << "===========================================\n";
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

// Display servo values in ncurses window for follower arm
void displayServoValues(WINDOW* win, const std::vector<ServoData>& arm2_data)
{
    werase(win);

    // Display header
    mvwprintw(win, 0, 0, "Perseus Follower Arm Servo Positions (0-4095) and Torque (-100 to +100)");
    mvwprintw(win, 1, 0, "--------------------------------------------------------");

    // Column headers
    mvwprintw(win, 2, 2, "Servo    Current    Min      Max      Torque  Mirroring");
    mvwprintw(win, 3, 0, "--------------------------------------------------------");

    // Display follower arm's servos
    mvwprintw(win, 4, 0, "Follower Arm:");
    for (size_t i = 0; i < 6; ++i)
    {
        int row = i + 5;
        const auto& servo = arm2_data[i];

        if (servo.error.empty())
        {
            if (servo.mirroring && has_colors())
            {
                wattron(win, COLOR_PAIR(4) | A_BOLD);
            }

            mvwprintw(win, row, 2, "%-8d %8u  %8u  %8u  [Torque: %5d] [%s]",
                      static_cast<int>(i + 1),
                      servo.current,
                      servo.min,
                      servo.max,
                      servo.torque,
                      servo.mirroring ? "Active" : "Inactive");

            // Display torque bar
            displayTorqueBar(win, row, 42, servo.torque);

            // Display position bar
            displayProgressBar(win, row, 49, servo.current, servo.min, servo.max);

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

    // Show network status
    if (follower_controller_ptr)
    {
        mvwprintw(win, 12, 0, "Network Status: %s",
                  follower_controller_ptr->getStatusMessage().c_str());
    }
    else
    {
        mvwprintw(win, 12, 0, "Network Status: Not initialized");
    }

    mvwprintw(win, 13, 0, "--------------------------------------------------------");

    // Add instructions
    mvwprintw(win, 14, 0, "Instructions:");
    mvwprintw(win, 15, 0, "1. Waiting for commands from leader");
    mvwprintw(win, 16, 0, "2. Press 't' to toggle torque protection (%s)",
              torque_protection.load() ? "ON" : "OFF");
    mvwprintw(win, 17, 0, "3. Press Ctrl+C to exit");

    mvwprintw(win, 20, 0, "Working directory: %s", getWorkingDirectory().c_str());

    wrefresh(win);
}

void disableTorqueAndCleanup()
{
    // First disable ncurses if it's active
    if (ncurses_win != nullptr)
    {
        mvwprintw(ncurses_win, 21, 0, "Disabling servo torque and cleaning up...");
        wrefresh(ncurses_win);
    }

    // Stop network controller
    if (follower_controller_ptr != nullptr)
    {
        follower_controller_ptr->stop();
    }

    // Disable torque for all servos on the arm
    if (reader2_ptr != nullptr)
    {
        for (uint8_t i = 1; i <= 6; ++i)
        {
            try
            {
                reader2_ptr->writeControlRegister(i, 0x28, 0);  // 0x28 is torque enable register
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

void showNetworkConfiguration(const std::string& bind_address, int port)
{
    std::cout << "\nNetwork Configuration:\n";
    std::cout << "=====================\n";
    std::cout << "Listening on: " << (bind_address.empty() ? "All interfaces" : bind_address) << ":" << port << "\n\n";
    std::cout << "Press Enter to continue...";
    std::cin.ignore(10000, '\n');
    std::string dummy;
    std::getline(std::cin, dummy);
}

int main(int argc, char* argv[])
{
    std::string port_path;
    std::string bind_address = "";  // empty means INADDR_ANY (all interfaces)
    int listen_port = arm_teleop::protocol::DEFAULT_PORT;

    try
    {
        // Set up signal handling
        signal(SIGINT, signalHandler);

        // Process command-line arguments for network configuration
        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];
            if ((arg == "--bind" || arg == "-b") && i + 1 < argc)
            {
                bind_address = argv[++i];
            }
            else if ((arg == "--port" || arg == "-p") && i + 1 < argc)
            {
                listen_port = std::stoi(argv[++i]);
            }
            else if ((arg == "--help" || arg == "-h"))
            {
                std::cout << "Usage: " << argv[0] << " [options]\n"
                          << "Options:\n"
                          << "  -b, --bind ADDRESS    Address to bind to (default: all interfaces)\n"
                          << "  -p, --port PORT       Port to listen on (default: " << arm_teleop::protocol::DEFAULT_PORT << ")\n"
                          << "  -h, --help            Show this help message\n";
                return 0;
            }
            else if (i == 1 && arg.find("tty") != std::string::npos)
            {
                // Assume first non-option argument is the serial port
                port_path = arg;
            }
        }

        showNetworkConfiguration(bind_address, listen_port);

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

        // Initialize servo reader for follower arm
        ST3215ServoReader reader2(port_path, 1000000, 30);  // 30 sets the acceleration
        reader2_ptr = &reader2;

        // Initialize network controller
        arm_teleop::FollowerController follower_controller(bind_address, listen_port);
        follower_controller_ptr = &follower_controller;

        // Set the servo controller
        follower_controller.setServoController(&reader2);

        // Initialize the network
        bool network_initialized = follower_controller.initialize();
        if (!network_initialized)
        {
            mvwprintw(ncurses_win, 21, 0, "Error: Failed to initialize network listener.");
            wrefresh(ncurses_win);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            disableTorqueAndCleanup();
            std::cerr << "Failed to initialize network. Check port availability." << std::endl;
            return 1;
        }

        // Start network thread
        follower_controller.start();

        // Initial display
        std::vector<ServoData> servo_data(6);
        displayServoValues(ncurses_win, servo_data);

        // Main loop
        while (running)
        {
            // Get the current servo data from the controller
            std::vector<ServoData> arm2_data = follower_controller.getServoData();

            // Update display
            displayServoValues(ncurses_win, arm2_data);

            // Handle keyboard input
            int ch = wgetch(ncurses_win);
            if (ch == 't' || ch == 'T')
            {
                bool new_state = !torque_protection.load();
                torque_protection.store(new_state);

                mvwprintw(ncurses_win, 21, 0, "                                                                        ");
                if (has_colors())
                {
                    wattron(ncurses_win, COLOR_PAIR(new_state ? 2 : 5) | A_BOLD);
                }
                mvwprintw(ncurses_win, 21, 0, "Torque protection %s", new_state ? "ENABLED" : "DISABLED");
                if (has_colors())
                {
                    wattroff(ncurses_win, COLOR_PAIR(new_state ? 2 : 5) | A_BOLD);
                }
                wrefresh(ncurses_win);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                mvwprintw(ncurses_win, 21, 0, "                                                                        ");
            }

            // Delay to prevent consuming too much CPU
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