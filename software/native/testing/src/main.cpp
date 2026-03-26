#include <chrono>
#include <hi_can_parameter.hpp>
#include <hi_can_raw.hpp>
#include <iostream>
#include <optional>
#include <thread>

using namespace std::chrono_literals;

using namespace hi_can;

std::optional<hi_can::RawCanInterface> can_interface;
std::optional<hi_can::PacketManager> packet_manager;

void setup()
{
    can_interface.emplace(RawCanInterface("vcan0"));
    packet_manager.emplace(can_interface.value());
    std::cout << "CAN interface initialized on vcan0" << std::endl;

    std::cout << "Setup complete - ready to receive CAN commands" << std::endl;
}

void loop()
{
    try
    {
        packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        std::cout << "CAN error: " << e.what() << std::endl;
    }
}

int main()
{
    setup();

    while (true)
    {
        loop();
        std::this_thread::sleep_for(1ms);
    }
}
