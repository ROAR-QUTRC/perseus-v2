#include <freertos/FreeRTOS.h>

#include <hi_can_twai.hpp>
void loop();
extern "C" void app_main()
{
    xCreateTaskPinnedToCore([](void* args)
                            {
    while(true) loop(args); }, "loop", 8096, NULL, 5, NULL, 1);
}

void loop()
{
}