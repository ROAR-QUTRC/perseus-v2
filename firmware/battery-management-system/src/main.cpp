#include <freertos/FreeRTOS.h>

#include <hi_can_twai.hpp>

#include "freertos/task.h"
#include "freertos/timers.h"

const int STACK_SIZE = 8096;
const int LOOP_PRIORITY = 5;
const int LOOP_CORE = 1;
std::optional<PacketManager> packet_manager;

void loop();
extern "C" void app_main()
{
    try
    {
        auto& can_interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                          filter_t{
                                                              .address = static_cast<flagged_address_t>(RCB_DEVICE_ADDRESS),
                                                              .mask = DEVICE_MASK,
                                                          });
        packet_manager.emplace(can_interface);
    }
    catch (const std::exception& e)
    {
        printf("Error \"%s\" while initialising CAN", e.what());
        return;
    }

    xTaskCreatePinnedToCore([](void*)
                            {
    while(true) loop(); }, "loop", STACK_SIZE, NULL, LOOP_PRIORITY, NULL, LOOP_CORE);
}

void loop()
{
}