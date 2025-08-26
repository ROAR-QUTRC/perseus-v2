#include <Arduino.h>
#include <driver/uart.h>
#include <nvs_flash.h>

#include <hi_can_twai.hpp>

const gpio_num_t RCB_POWER_LED_PIN = GPIO_NUM_12;
const gpio_num_t RCB_CONTACTOR_PIN = GPIO_NUM_13;

const gpio_num_t RCB_SPARE_PRE_SWITCH_PIN = GPIO_NUM_41;
const gpio_num_t RCB_SPARE_MAIN_SWITCH_PIN = GPIO_NUM_40;

const gpio_num_t RCB_DRIVE_PRE_SWITCH_PIN = GPIO_NUM_39;
const gpio_num_t RCB_DRIVE_MAIN_SWITCH_PIN = GPIO_NUM_38;

const gpio_num_t RCB_COMP_PRE_SWITCH_PIN = GPIO_NUM_37;
const gpio_num_t RCB_COMP_MAIN_SWITCH_PIN = GPIO_NUM_45;

const gpio_num_t RCB_AUX_PRE_SWITCH_PIN = GPIO_NUM_48;
const gpio_num_t RCB_AUX_MAIN_SWITCH_PIN = GPIO_NUM_47;

void setup()
{
    esp_rom_gpio_pad_select_gpio(RCB_CONTACTOR_PIN);
    gpio_set_direction(RCB_CONTACTOR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_intr_type(RCB_CONTACTOR_PIN, GPIO_INTR_DISABLE);

    esp_rom_gpio_pad_select_gpio(RCB_POWER_LED_PIN);
    gpio_set_direction(RCB_POWER_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_intr_type(RCB_POWER_LED_PIN, GPIO_INTR_DISABLE);

    fflush(stdout);
    printf("\033[0m\r\n");
    printf("\r\n-----------------------------------------------------------------\r\n");

    // UART Init
    if (uart_is_driver_installed(UART_NUM_0))
        ESP_ERROR_CHECK(uart_driver_delete(UART_NUM_0));
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 1024, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, -1, -1, -1, -1));

    // NVS Init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void loop()
{
}