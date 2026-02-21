#include "rover_adc.hpp"

// core libs
#include <driver/gpio.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>

// standard libs
#include <atomic>
#include <cstring>

// rover libs
#include <rover_core.hpp>
#include <rover_log.hpp>

static const uint8_t ADC_MAX_CHANNELS = 10;
static const uint8_t ADC_OVERSAMPLE_COUNT = 50;
static const uint8_t ADC_ERROR_SAMPLE_COUNT = 20;  // MUST be less than ADC_OVERSAMPLE_COUNT

using std::atomic;

static bool initialised = false;

static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_calibration_handle = NULL;

static uint8_t result_index_count = 0;
static uint8_t result_indices[ADC_MAX_CHANNELS] = {0};

static atomic<bool> has_new_data[ADC_MAX_CHANNELS];
static atomic<uint16_t> adc_voltages[ADC_MAX_CHANNELS];

static uint32_t adc_accumulators[ADC_MAX_CHANNELS];
static uint8_t adc_accumulator_counters[ADC_MAX_CHANNELS];

static uint16_t adc_error_levels[ADC_MAX_CHANNELS];
static uint8_t adc_error_counters[ADC_MAX_CHANNELS];

static int adc_reading_to_voltage(int raw)
{
    int voltage = 0;
    if (adc_cali_raw_to_voltage(adc_calibration_handle, raw, &voltage) != ESP_OK)
    {
        voltage = (raw * 3100UL) / ((1UL << 12) - 1);
    }
    return (int32_t)voltage;
}

static int pin_to_adc_index(gpio_num_t gpio)
{
    adc_unit_t unit;
    adc_channel_t channel = ADC_CHANNEL_0;
    adc_continuous_io_to_channel(gpio, &unit, &channel);
    return channel;
}

static bool is_pin_valid_adc(gpio_num_t gpio, const char* error_message = nullptr, adc_channel_t* out_channel = nullptr)
{
    adc_unit_t unit;
    adc_channel_t channel;
    if ((adc_continuous_io_to_channel(gpio, &unit, &channel) != ESP_OK) && (unit != ADC_UNIT_1))
    {
        if (error_message)
        {
            printf(error_message, gpio);
        }
        return false;
    }

    if (out_channel)
        *out_channel = channel;

    return true;
}

bool adc_conversion_complete_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata, void* user_data)
{
    const uint8_t result_count = (edata->size / 4);
    for (int i = 0; i < result_count; i++)
    {
        const adc_digi_output_data_t result = ((adc_digi_output_data_t*)edata->conv_frame_buffer)[i];
        const uint16_t voltage = adc_reading_to_voltage(result.type2.data);
        const int index = result_indices[i];

        if (voltage > adc_error_levels[index])
        {
            adc_error_counters[index]++;
        }
        else
        {
            adc_accumulators[index] += voltage;
        }

        adc_accumulator_counters[index]++;
        if (adc_accumulator_counters[index] >= ADC_OVERSAMPLE_COUNT)
        {
            if (adc_error_counters[index] > ADC_ERROR_SAMPLE_COUNT)
                adc_voltages[index].store(ROVER_ADC_ERR_RETURN_VAL);
            else
                adc_voltages[index].store(adc_accumulators[index] / adc_accumulator_counters[index]);

            has_new_data[index].store(true);

            adc_accumulator_counters[index] = 0;
            adc_accumulators[index] = 0;
            adc_error_counters[index] = 0;
        }
    }
    return false;
}

void adc_deinitialize()
{
    if (!initialised)
    {
        printf("ADC not initialised, cannot de-init!");
        return;
    }
    printf("De-initialising ADC");

    adc_continuous_stop(adc_handle);
    adc_continuous_deinit(adc_handle);
    adc_cali_delete_scheme_curve_fitting(adc_calibration_handle);

    memset(adc_accumulators, 0, sizeof(adc_accumulators));
    memset(adc_accumulator_counters, 0, sizeof(adc_accumulator_counters));
    for (int i = 0; i < ADC_MAX_CHANNELS; i++)
    {
        has_new_data[i].store(false);
        adc_voltages[i].store(0);
    }

    initialised = false;
    printf("ADC de-initialised");
}

void adc_set_error_voltage(gpio_num_t gpio, uint16_t errorVtg)
{
    if (!is_pin_valid_adc(gpio, "Error setting pin %d error voltage - is not valid ADC channel!"))
        return;

    int index = pin_to_adc_index(gpio);
    printf("ADC channel %d error voltage now %u", index, errorVtg);
    adc_error_levels[index] = errorVtg;
}
void adc_set_channel_enabled(gpio_num_t gpio, bool enable, bool delayedInit)
{
    static bool is_initialized[ADC_MAX_CHANNELS]{};

    adc_channel_t channel;
    if (!is_pin_valid_adc(gpio, "Error configuring pin %d - is not valid ADC channel!", &channel))
        return;

    printf("%s ADC channel %d, pin %d", enable ? "Enabling" : "Disabling", channel, gpio);

    int index = pin_to_adc_index(gpio);
    is_initialized[index] = enable;
    adc_accumulator_counters[index] = 0;
    adc_accumulators[index] = 0;
    adc_error_counters[index] = 0;
    adc_error_levels[index] = ROVER_ADC_DISABLE_ERROR;
    if (delayedInit)
        return;

    adc_digi_pattern_config_t adc_conversion_config[ADC_MAX_CHANNELS]{};
    result_index_count = 0;
    for (int i = 0; i < ADC_MAX_CHANNELS; i++)
    {
        if (is_initialized[i])
        {
            adc_conversion_config[result_index_count] = {
                .atten = ADC_ATTEN_DB_12,
                .channel = (uint8_t)i,
                .unit = ADC_UNIT_1,
                .bit_width = 12,
            };
            result_indices[result_index_count] = i;
            result_index_count++;
        }
    }

    if (initialised)
        adc_deinitialize();

    if (result_index_count == 0)
    {
        printf("No channels enabled!");
        return;  // if no channels enabled, nothing to do!
    }

    adc_cali_curve_fitting_config_t adc_calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_0,  // not used! provided for extensibility as of ESP-IDF v5.1.1
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    // non-returning err check since we can do a linear estimation even if calibrated conversion fails
    ROVER_CORE_ERR_CHECK("ADC calibration scheme creation failed!",
                         adc_cali_create_scheme_curve_fitting(&adc_calibration_config, &adc_calibration_handle));

    printf("%d ADC channels enabled - configuring...", result_index_count);
    adc_continuous_handle_cfg_t adcConfig = {
        .max_store_buf_size = 4U * result_index_count,
        .conv_frame_size = 4U * result_index_count,
        .flags = {
            .flush_pool = true,
        },
    };
    ROVER_CORE_RET_ERR_CHECK("ADC initialisation failed!", adc_continuous_new_handle(&adcConfig, &adc_handle));

    adc_continuous_config_t adcChanConfig = {
        .pattern_num = result_index_count,
        .adc_pattern = adc_conversion_config,
        .sample_freq_hz = 1000 * ADC_OVERSAMPLE_COUNT,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,     // we only use unit 1
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,  // only type2 is valid
    };
    ROVER_CORE_RET_ERR_CHECK("Error configuring ADC!", adc_continuous_config(adc_handle, &adcChanConfig));

    adc_continuous_evt_cbs_t adcCallbacks = {
        .on_conv_done = adc_conversion_complete_callback,
        .on_pool_ovf = NULL,
    };
    ROVER_CORE_RET_ERR_CHECK("Error registering ADC callbacks!",
                             adc_continuous_register_event_callbacks(adc_handle, &adcCallbacks, NULL));
    ROVER_CORE_RET_ERR_CHECK("Error starting ADC!", adc_continuous_start(adc_handle));

    initialised = true;

    printf("ADC initialisation complete");
}

int32_t adc_get_voltage(gpio_num_t gpio)
{
    if (!is_pin_valid_adc(gpio, "Attempting to read ADC value for un-managed pin %d!"))
        return 0;

    const int index = pin_to_adc_index(gpio);
    has_new_data[index].store(false);
    return adc_voltages[index].load();
}