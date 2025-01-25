#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "bmi08_defs.h"
#include "bmi08.h"
#include "bmi08x.h"
#include "bmi088_anymotion.h"
#include "bmi088_mm.h"

// Define SPI pins
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS_ACCEL   5
#define PIN_NUM_CS_GYRO    4

// Add after existing includes
#include "esp_log.h"

// Add after existing defines
#define TAG "BMI088"
#define SENSOR_READ_DELAY_MS 100

// Global device structure
struct bmi08_dev bmi08dev;

// SPI device handle
spi_device_handle_t spi;

// Function prototypes
esp_err_t init_spi(void);
int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

// Add these functions before app_main()
void read_sensor_data(void *pvParameters) {
    struct bmi08_sensor_data accel;
    struct bmi08_sensor_data gyro;
    int8_t rslt;
    while (1) {
        // Read accelerometer data
        rslt = bmi08a_get_data(&accel, &bmi08dev);
        if (rslt != BMI08_OK) {
            printf("Failed to read accelerometer data\n");
        }

        // Read gyroscope data
        rslt = bmi08g_get_data(&gyro, &bmi08dev);
        if (rslt != BMI08_OK) {
            printf("Failed to read gyroscope data\n");
        }

        // Print sensor data
        printf("Accel: X=%6.2f Y=%6.2f Z=%6.2f m/s²\n", 
            accel.x * 9.81f / 32768.0f,
            accel.y * 9.81f / 32768.0f, 
            accel.z * 9.81f / 32768.0f);
        
        printf("Gyro:  X=%6.2f Y=%6.2f Z=%6.2f deg/s\n",
            gyro.x * 2000.0f / 32768.0f,
            gyro.y * 2000.0f / 32768.0f,
            gyro.z * 2000.0f / 32768.0f);

        vTaskDelay(SENSOR_READ_DELAY_MS);
    }
}

void app_main(void) {
    printf("Initializing BMI088...\n");

    // Initialize SPI
    ESP_ERROR_CHECK(init_spi());

    // Configure BMI088
    bmi08dev.intf = BMI08_SPI_INTF;
    bmi08dev.variant = BMI088_VARIANT;
    bmi08dev.read = user_spi_read;
    bmi08dev.write = user_spi_write;
    bmi08dev.delay_us = (void *)vTaskDelay;
    bmi08dev.intf_ptr_accel = &spi;
    bmi08dev.intf_ptr_gyro = &spi;

    // Initialize accelerometer
    int8_t rslt = bmi08xa_init(&bmi08dev);
    if (rslt != BMI08_OK) {
        printf("Failed to initialize accelerometer! Error code: %d\n", rslt);
        return;
    }

    // Initialize gyroscope
    rslt = bmi08g_init(&bmi08dev);
    if (rslt != BMI08_OK) {
        printf("Failed to initialize gyroscope! Error code: %d\n", rslt);
        return;
    }

    // Configure accelerometer
    bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
    bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;
    bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;

    rslt = bmi08a_set_power_mode(&bmi08dev);
    if (rslt != BMI08_OK) {
        printf("Failed to set accelerometer power mode! Error code: %d\n", rslt);
        return;
    }

    // Configure gyroscope
    bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_23_ODR_200_HZ;
    bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_23_ODR_200_HZ;

    rslt = bmi08g_set_power_mode(&bmi08dev);
    if (rslt != BMI08_OK) {
        printf("Failed to set gyroscope power mode! Error code: %d\n", rslt);
        return;
    }

    printf("BMI088 initialization complete!\n");

    // Create sensor reading task
    xTaskCreate(read_sensor_data, "bmi088_reader", 4096, NULL, 5, NULL);
}

esp_err_t init_spi(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1 MHz
        .mode = 0,                  // SPI mode 0
        .spics_io_num = -1,        // CS pin handled manually
        .queue_size = 7,
    };

    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
    // Add device to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi));

    // Configure CS pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CS_ACCEL) | (1ULL << PIN_NUM_CS_GYRO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Set CS pins high (inactive)
    gpio_set_level(PIN_NUM_CS_ACCEL, 1);
    gpio_set_level(PIN_NUM_CS_GYRO, 1);

    return ESP_OK;
}

int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t spi_dev = *(spi_device_handle_t*)intf_ptr;
    
    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * (len + 1),  // Total length in bits
        .tx_buffer = &reg_addr,
        .rx_buffer = reg_data
    };
    
    esp_err_t ret = spi_device_transmit(spi_dev, &t);
    return (ret == ESP_OK) ? BMI08_OK : BMI08_E_COM_FAIL;
}

int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t spi_dev = *(spi_device_handle_t*)intf_ptr;
    
    uint8_t tx_buffer[len + 1];
    tx_buffer[0] = reg_addr;
    memcpy(&tx_buffer[1], reg_data, len);
    
    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * (len + 1),  // Total length in bits
        .tx_buffer = tx_buffer
    };
    
    esp_err_t ret = spi_device_transmit(spi_dev, &t);
    return (ret == ESP_OK) ? BMI08_OK : BMI08_E_COM_FAIL;
}
