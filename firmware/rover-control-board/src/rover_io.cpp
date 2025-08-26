#include "rover_io.hpp"

// drivers
#include <nvs_flash.h>

// libraries
#include <rover_log.hpp>
// #include "esp_littlefs.h"

void ioInitUart(uart_port_t uart, int baudRate, int pinTx, int pinRx)
{
    if (uart_is_driver_installed(uart))
        ESP_ERROR_CHECK(uart_driver_delete(uart));
    uart_config_t uart_config = {
        .baud_rate           = baudRate,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk          = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(uart, CONFIG_UART_RX_BUFFER_SIZE, CONFIG_UART_TX_BUFFER_SIZE, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart, pinTx, pinRx, -1, -1));  //*/
    IO_DEBUG("Initialised uart %d to %d baud", (int)uart, baudRate);
}

void ioInitNvs()
{
    IO_DEBUG("Initialising NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        IO_DEBUG("NVS init failed! Erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void ioInitFiles()
{
    // IO_INFO("Mounting LittleFS filesystem to /" CONFIG_FILESYS_ROOT_PATH);
    /*esp_vfs_littlefs_conf_t littlefs_conf = {
        .base_path = CONFIG_FILESYS_ROOT_PATH,
        .partition_label = "files",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    ESP_ERROR_CHECK(esp_vfs_littlefs_register(&littlefs_conf));//*/
}
void ioStopFiles()
{
    // IO_INFO("Stopping filesystem");
    // esp_vfs_littlefs_unregister("files");
}

void ioConfigInput(gpio_num_t gpio, gpio_pull_mode_t pull)
{
    esp_rom_gpio_pad_select_gpio(gpio);

    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_set_intr_type(gpio, GPIO_INTR_DISABLE);
    gpio_set_pull_mode(gpio, pull);
}
void ioConfigOutput(gpio_num_t gpio, bool openDrain)
{
    esp_rom_gpio_pad_select_gpio(gpio);

    gpio_set_direction(gpio, openDrain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT);
    gpio_set_intr_type(gpio, GPIO_INTR_DISABLE);
}