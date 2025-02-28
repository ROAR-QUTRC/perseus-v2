#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
// Remove esp_log.h since we're not using it anymore
// #include "esp_log.h"

// SPI Configuration
#define SPI_HOST    SPI2_HOST  // SPI2_HOST is the new name for HSPI_HOST
#define PIN_MISO    GPIO_NUM_13
#define PIN_MOSI    GPIO_NUM_14
#define PIN_CLK     GPIO_NUM_12
#define PIN_CS_ACC  GPIO_NUM_11   // Accelerometer CS
#define PIN_CS_GYRO GPIO_NUM_15   // Gyroscope CS

// BMI088 registers
#define BMI088_ACC_CHIP_ID        0x1E
#define BMI088_ACC_ERR_REG        0x02
#define BMI088_ACC_STATUS         0x03
#define BMI088_ACC_X_LSB          0x12
#define BMI088_ACC_PWR_CTRL       0x7D
#define BMI088_ACC_PWR_CONF       0x7C
#define BMI088_ACC_SOFTRESET      0x7E

#define BMI088_GYRO_CHIP_ID       0x00
#define BMI088_GYRO_RATE_X_LSB    0x02
#define BMI088_GYRO_RANGE         0x0F
#define BMI088_GYRO_BANDWIDTH     0x10
#define BMI088_GYRO_LPM1          0x11
#define BMI088_GYRO_SOFTRESET     0x14

// Constants
#define BMI088_ACC_CHIP_ID_VAL    0x1E
#define BMI088_GYRO_CHIP_ID_VAL   0x0F

// We'll keep TAG variable for message prefixing
static const char *TAG = "BMI088";
static spi_device_handle_t acc_spi;
static spi_device_handle_t gyro_spi;

// Function prototypes
static esp_err_t bmi088_init(void);
static esp_err_t bmi088_acc_read(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t bmi088_acc_write(uint8_t reg_addr, uint8_t data);
static esp_err_t bmi088_gyro_read(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t bmi088_gyro_write(uint8_t reg_addr, uint8_t data);
static void bmi088_read_sensor_data(void *pvParameters);

// Include UART related headers
#include "driver/uart.h"
#include "esp_vfs_dev.h"

// Setup UART for printf output
static void uart_init(void) {
    // Configure UART for USB output
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
}

// Initialize SPI
static esp_err_t spi_init(void) {
    esp_err_t ret;
    
    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("[%s] Failed to initialize SPI bus\n", TAG);
        return ret;
    }
    
    // Accelerometer SPI device configuration
    spi_device_interface_config_t acc_devcfg = {
        .clock_speed_hz = 10*1000*1000,    // 10 MHz
        .mode = 0,                         // SPI mode 0
        .spics_io_num = PIN_CS_ACC,
        .queue_size = 7,
        .command_bits = 0,
        .address_bits = 0,
    };
    
    ret = spi_bus_add_device(SPI_HOST, &acc_devcfg, &acc_spi);
    if (ret != ESP_OK) {
        printf("[%s] Failed to add accelerometer SPI device\n", TAG);
        return ret;
    }
    
    // Gyroscope SPI device configuration
    spi_device_interface_config_t gyro_devcfg = {
        .clock_speed_hz = 10*1000*1000,    // 10 MHz
        .mode = 0,                         // SPI mode 0
        .spics_io_num = PIN_CS_GYRO,
        .queue_size = 7,
        .command_bits = 0,
        .address_bits = 0,
    };
    
    ret = spi_bus_add_device(SPI_HOST, &gyro_devcfg, &gyro_spi);
    if (ret != ESP_OK) {
        printf("[%s] Failed to add gyroscope SPI device\n", TAG);
        return ret;
    }
    
    return ESP_OK;
}

// Read from accelerometer registers
static esp_err_t bmi088_acc_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t ret;
    spi_transaction_t t = {0};
    
    // For accelerometer reads: bit 7 should be set to 1
    uint8_t tx_data[2] = {reg_addr | 0x80, 0x00};  // Address with read bit + dummy byte
    uint8_t rx_data[2] = {0};
    
    // Send register address and dummy byte
    t.length = 16;  // 2 bytes (8 bits each)
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    ret = spi_device_polling_transmit(acc_spi, &t);
    if (ret != ESP_OK) return ret;
    
    // Now read the actual data
    t.length = 8 * len;
    t.tx_buffer = NULL;
    t.rx_buffer = data;
    ret = spi_device_polling_transmit(acc_spi, &t);
    
    return ret;
}

// Write to accelerometer registers
static esp_err_t bmi088_acc_write(uint8_t reg_addr, uint8_t data) {
    esp_err_t ret;
    spi_transaction_t t = {0};
    
    // For accelerometer writes: bit 7 should be clear (0)
    uint8_t tx_data[2] = {reg_addr & 0x7F, data};
    
    t.length = 16;
    t.tx_buffer = tx_data;
    t.rx_buffer = NULL;
    ret = spi_device_polling_transmit(acc_spi, &t);
    
    return ret;
}

// Read from gyroscope registers
static esp_err_t bmi088_gyro_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t ret;
    spi_transaction_t t = {0};
    
    // For gyroscope reads: bit 7 should be set to 1
    uint8_t tx_data = reg_addr | 0x80;
    
    // Send register address
    t.length = 8;
    t.tx_buffer = &tx_data;
    t.rx_buffer = NULL;
    ret = spi_device_polling_transmit(gyro_spi, &t);
    if (ret != ESP_OK) return ret;
    
    // Read data
    t.length = 8 * len;
    t.tx_buffer = NULL;
    t.rx_buffer = data;
    ret = spi_device_polling_transmit(gyro_spi, &t);
    
    return ret;
}

// Write to gyroscope registers
static esp_err_t bmi088_gyro_write(uint8_t reg_addr, uint8_t data) {
    esp_err_t ret;
    spi_transaction_t t = {0};
    
    // For gyroscope writes: bit 7 should be clear (0)
    uint8_t tx_data[2] = {reg_addr & 0x7F, data};
    
    t.length = 16;
    t.tx_buffer = tx_data;
    t.rx_buffer = NULL;
    ret = spi_device_polling_transmit(gyro_spi, &t);
    
    return ret;
}

// Initialize BMI088 sensor
static esp_err_t bmi088_init(void) {
    esp_err_t ret;
    uint8_t chip_id;
    
    // Initialize SPI
    ret = spi_init();
    if (ret != ESP_OK) return ret;
    
    // Reset accelerometer
    ret = bmi088_acc_write(BMI088_ACC_SOFTRESET, 0xB6);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100)); // Increase to 100ms
    
    // Read accelerometer chip ID
    ret = bmi088_acc_read(BMI088_ACC_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) return ret;
    
    printf("[%s] Accelerometer chip ID: 0x%02x\n", TAG, chip_id);
    if (chip_id != BMI088_ACC_CHIP_ID_VAL) {
        printf("[%s] Accelerometer chip ID mismatch!\n", TAG);
        return ESP_FAIL;
    }
    
    // Configure accelerometer
    // Power on accelerometer
    ret = bmi088_acc_write(BMI088_ACC_PWR_CONF, 0x00); // Active mode
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    ret = bmi088_acc_write(BMI088_ACC_PWR_CTRL, 0x04); // Enable accelerometer
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Reset gyroscope
    ret = bmi088_gyro_write(BMI088_GYRO_SOFTRESET, 0xB6);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for reset
    
    // Read gyroscope chip ID
    ret = bmi088_gyro_read(BMI088_GYRO_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) return ret;
    
    printf("[%s] Gyroscope chip ID: 0x%02x\n", TAG, chip_id);
    if (chip_id != BMI088_GYRO_CHIP_ID_VAL) {
        printf("[%s] Gyroscope chip ID mismatch!\n", TAG);
        return ESP_FAIL;
    }
    
    // Configure gyroscope
    ret = bmi088_gyro_write(BMI088_GYRO_RANGE, 0x01); // ±500 degrees/s
    if (ret != ESP_OK) return ret;
    
    ret = bmi088_gyro_write(BMI088_GYRO_BANDWIDTH, 0x02); // ODR: 1000Hz, Filter: 116Hz
    if (ret != ESP_OK) return ret;
    
    ret = bmi088_gyro_write(BMI088_GYRO_LPM1, 0x00); // Normal mode
    if (ret != ESP_OK) return ret;
    
    printf("[%s] BMI088 initialization complete\n", TAG);
    
    return ESP_OK;
}

// Read sensor data (accelerometer and gyroscope)
static void bmi088_read_sensor_data(void *pvParameters) {
    uint8_t data[6];
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float acc_scale = 0.00122f; // For ±12g range
    float gyro_scale = 0.0153f; // For ±500 degrees/s range
    
    while (1) {
        // Read accelerometer data
        if (bmi088_acc_read(BMI088_ACC_X_LSB, data, 6) == ESP_OK) {
            // BMI088 accelerometer data: LSB first, then MSB
            accel_x = (int16_t)((data[1] << 8) | data[0]);
            accel_y = (int16_t)((data[3] << 8) | data[2]);
            accel_z = (int16_t)((data[5] << 8) | data[4]);
            
            float acc_x = accel_x * acc_scale;
            float acc_y = accel_y * acc_scale;
            float acc_z = accel_z * acc_scale;
            
            printf("[%s] Accel: X=%.2f g, Y=%.2f g, Z=%.2f g\n", TAG, acc_x, acc_y, acc_z);
        }
        
        // Read gyroscope data
        if (bmi088_gyro_read(BMI088_GYRO_RATE_X_LSB, data, 6) == ESP_OK) {
            // BMI088 gyroscope data: LSB first, then MSB
            gyro_x = (int16_t)((data[1] << 8) | data[0]);
            gyro_y = (int16_t)((data[3] << 8) | data[2]);
            gyro_z = (int16_t)((data[5] << 8) | data[4]);
            
            float gyr_x = gyro_x * gyro_scale;
            float gyr_y = gyro_y * gyro_scale;
            float gyr_z = gyro_z * gyro_scale;
            
            printf("[%s] Gyro: X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s\n", TAG, gyr_x, gyr_y, gyr_z);
        }
        
        // Wait before next reading
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void hello_task(void *pvParameters) {
    while (1) {
        printf("[%s] Failed to initialize BMI088 sensor\n", TAG);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

void app_main(void) {
    // Initialize UART for console output
    uart_init();

    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 1 second

    printf("[%s] Starting BMI088 sensor application\n", TAG);
    
    // Initialize the BMI088 sensor
    if (bmi088_init() == ESP_OK) {
        // Create a task to read sensor data periodically
        xTaskCreate(bmi088_read_sensor_data, "bmi088_read", 4096, NULL, 5, NULL);
    } else {
        printf("[%s] Failed to initialize BMI088 sensor\n", TAG);
            // Create task for printing hello world
        xTaskCreate(hello_task, "hello_task", 2048, NULL, 5, NULL);
    }
}