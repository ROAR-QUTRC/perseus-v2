#include "main.hpp"

#include <board_support.hpp>
#include <hi_can_twai.hpp>

using namespace hi_can;
using namespace hi_can::addressing;
void loop(void* args);
extern "C" void app_main()
{
    TwaiInterface& interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                           addressing::filter_t{
                                                               .address = static_cast<raw_address_t>(standard_address_t(power::SYSTEM_ID, power::battery::SUBSYSTEM_ID, static_cast<uint8_t>(power::battery::device::BMS), 0x00, 0x00)),
                                                               .mask = hi_can::addressing::DEVICE_MASK,
                                                           });
    PacketManager packet_manager(interface);

    xTaskCreatePinnedToCore([](void* args)
                            { while(true) loop(args); }, "loop", 8192,
                            NULL, 10, NULL, 1);
}

void loop(void* args)
{
}