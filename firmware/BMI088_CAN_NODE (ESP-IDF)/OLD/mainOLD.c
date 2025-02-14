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
#include "esp_heap_caps.h"
#include "driver/uart.h"

// UART configuration
#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        GPIO_NUM_43  // ESP32-S3 default UART pins
#define UART_RX_PIN        GPIO_NUM_44
#define BUF_SIZE          1024


// Define SPI pins
#define PIN_NUM_MISO GPIO_NUM_3
#define PIN_NUM_MOSI GPIO_NUM_4
#define PIN_NUM_CLK  GPIO_NUM_2
#define PIN_NUM_CS_ACCEL GPIO_NUM_1
#define PIN_NUM_CS_GYRO  GPIO_NUM_5

// SPI configuration
#define SPI_HOST          SPI2_HOST
#define MAX_TRANSFER_SIZE 64
#define DMA_CHAN          2

// BMI088 Register addresses
#define BMI08X_ACC_CHIP_ID_REG 0x00
#define BMI08X_GYRO_CHIP_ID_REG 0x00

// Add after existing includes
#include "esp_log.h"

// Add after existing defines
#define TAG "BMI088"
#define SENSOR_READ_DELAY_MS 100

// Define SPI buffers
static uint8_t *tx_buffer = NULL;
static uint8_t *rx_buffer = NULL;

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
    // Allocate SPI buffers
    // Allocate DMA-capable memory
    tx_buffer = heap_caps_malloc(MAX_TRANSFER_SIZE, MALLOC_CAP_DMA);
    rx_buffer = heap_caps_malloc(MAX_TRANSFER_SIZE, MALLOC_CAP_DMA);
    
    ESP_LOGI(TAG, "Initializing BMI088...");

    ESP_ERROR_CHECK(init_spi());
    ESP_LOGI(TAG, "SPI initialized successfully");

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
        ESP_LOGE(TAG, "Failed to initialize accelerometer! Error code: %d", rslt);
        // Add chip ID verification
        uint8_t chip_id;
        rslt = user_spi_read(BMI08X_ACC_CHIP_ID_REG, &chip_id, 1, &spi);
        ESP_LOGI(TAG, "Accelerometer chip ID: 0x%02x", chip_id);
        return;
    }
    ESP_LOGI(TAG, "Accelerometer initialized successfully");


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
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10000000,    // 10MHz
        .mode = 0,
        .spics_io_num = -1,          // CS pin handled manually
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY
    };

    // Initialize SPI bus with DMA channel auto-allocation
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    // Configure CS pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CS_ACCEL) | (1ULL << PIN_NUM_CS_GYRO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    gpio_config(&io_conf);
    gpio_set_level(PIN_NUM_CS_ACCEL, 1);
    gpio_set_level(PIN_NUM_CS_GYRO, 1);

    return ESP_OK;
}

int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (!reg_data || !intf_ptr || len > 63) {
        return BMI08_E_NULL_PTR;
    }

    spi_device_handle_t spi = (spi_device_handle_t)intf_ptr;
    spi_transaction_t t = {
        .flags = 0,
        .length = (len + 1) * 8,
        .rxlength = (len + 1) * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };

    // Setup read command
    tx_buffer[0] = reg_addr | 0x80;
    memset(&tx_buffer[1], 0, len);

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) {
        return BMI08_E_COM_FAIL;
    }

    memcpy(reg_data, &rx_buffer[1], len);
    return BMI08_OK;
}

int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // Determine if this is accel or gyro access
    bool is_accel = (reg_addr & 0x80) == 0;
    gpio_num_t cs_pin = is_accel ? PIN_NUM_CS_ACCEL : PIN_NUM_CS_GYRO;

    spi_device_handle_t spi_dev = (spi_device_handle_t)intf_ptr;

    // Assert CS
    gpio_set_level(cs_pin, 0);

    uint8_t tx_buffer[len + 1];
    tx_buffer[0] = reg_addr & ~0x80; // Clear read bit
    memcpy(&tx_buffer[1], reg_data, len);

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * (len + 1),
        .tx_buffer = tx_buffer
    };

    esp_err_t ret = spi_device_transmit(spi_dev, &t);

    // Deassert CS
    gpio_set_level(cs_pin, 1);

    return (ret == ESP_OK) ? BMI08_OK : BMI08_E_COM_FAIL;
}