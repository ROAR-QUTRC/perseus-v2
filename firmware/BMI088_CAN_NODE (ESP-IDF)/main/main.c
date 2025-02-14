#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include "bmi08_defs.h"
#include "bmi08.h"
#include "bmi08x.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "soc/spi_periph.h"
#include "soc/spi_reg.h"
#include "esp_rom_sys.h"
#include "esp_rom_gpio.h"

// BMI088 Register definitions
#define BMI08_ACCEL_CHIP_ID_REG     0x00

// SPI Configuration
#define SPI_HOST    SPI2_HOST  // SPI2_HOST is the new name for HSPI_HOST
#define PIN_MISO    GPIO_NUM_13
#define PIN_MOSI    GPIO_NUM_14
#define PIN_CLK     GPIO_NUM_12
#define PIN_CS_ACC  GPIO_NUM_11   // Accelerometer CS
#define PIN_CS_GYRO GPIO_NUM_15   // Gyroscope CS

static spi_device_handle_t spi_acc;
static spi_device_handle_t spi_gyro;
struct bmi08_dev bmi08dev;

// SPI read/write functions for BMI088
BMI08_INTF_RET_TYPE bmi08_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_device_handle_t spi = (spi_device_handle_t)intf_ptr;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.length = 8 * (len + 1);
    t.rxlength = 8 * len;
    uint8_t tx_data[len + 1];
    tx_data[0] = reg_addr | 0x80; // Set read bit
    
    t.tx_buffer = tx_data;
    t.rx_buffer = reg_data;
    
    esp_err_t ret = spi_device_transmit(spi, &t);
    return (ret == ESP_OK) ? BMI08_OK : BMI08_E_COM_FAIL;
}

BMI08_INTF_RET_TYPE bmi08_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_device_handle_t spi = (spi_device_handle_t)intf_ptr;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.length = 8 * (len + 1);
    uint8_t tx_data[len + 1];
    tx_data[0] = reg_addr & 0x7F; // Clear read bit
    memcpy(&tx_data[1], reg_data, len);
    
    t.tx_buffer = tx_data;
    
    esp_err_t ret = spi_device_transmit(spi, &t);
    return (ret == ESP_OK) ? BMI08_OK : BMI08_E_COM_FAIL;
}

void bmi08_delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);
}

void init_spi(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10*1000*1000,
        .mode = 0,
        .spics_io_num = -1, // CS handled manually
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi_acc));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi_gyro));

    // Configure CS pins
    gpio_set_direction(PIN_CS_ACC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_CS_GYRO, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CS_ACC, 0);
    gpio_set_level(PIN_CS_GYRO, 0);
}

int app_main(void)
{
    printf("MAIN APP START...\n");
    
    // Initialize SPI
    init_spi();
    
    // Configure BMI088
    bmi08dev.intf = BMI08_SPI_INTF;
    bmi08dev.read = bmi08_spi_read;
    bmi08dev.write = bmi08_spi_write;
    bmi08dev.delay_us = bmi08_delay_us;
    bmi08dev.intf_ptr_accel = spi_acc;
    bmi08dev.intf_ptr_gyro = spi_gyro;
    
    // Initialize BMI088
    int8_t rslt = bmi08a_init(&bmi08dev);
    
    if (rslt != BMI08_OK) {
        printf("Could not initialize accelerometer!!!\n");
        return -1;
    }

    printf("Accelerometer initialized...\n");

    rslt = bmi08g_init(&bmi08dev);
    if (rslt != BMI08_OK) {
        printf("Could not initialize gyroscope!!!\n");
        //return -1;
    }

    //printf("Gyroscope initialized...\n");

    // Configure accelerometer
    bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
    bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;
    bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
    
    rslt = bmi08a_set_power_mode(&bmi08dev);
    rslt = bmi08a_set_meas_conf(&bmi08dev);

    // Configure gyroscope
    bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_32_ODR_100_HZ;
    bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_32_ODR_100_HZ;
    
    rslt = bmi08g_set_power_mode(&bmi08dev);
    
    // Main loop to read sensor data
    while(1) {
        struct bmi08_sensor_data accel;
        struct bmi08_sensor_data gyro;
        
        bmi08a_get_data(&accel, &bmi08dev);
        bmi08g_get_data(&gyro, &bmi08dev);
        
        printf("Acc: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n",
               accel.x, accel.y, accel.z,
               gyro.x, gyro.y, gyro.z);
               
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
