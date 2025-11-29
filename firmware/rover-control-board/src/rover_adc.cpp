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

// defines
#define ADC_MAX_CHANNELS       10
#define ADC_OVERSAMPLE_COUNT   50
#define ADC_ERROR_SAMPLE_COUNT 20  // MUST be less than ADC_OVERSAMPLE_COUNT

using std::atomic;

static bool initialised = false;

static adc_continuous_handle_t adcHandle = NULL;
static adc_cali_handle_t adcCaliHandle = NULL;

uint8_t resultIndexCount = 0;
uint8_t resultIndices[ADC_MAX_CHANNELS] = {0};

static atomic<bool> hasNewData[ADC_MAX_CHANNELS];
static atomic<uint16_t> adcVoltages[ADC_MAX_CHANNELS];

static uint32_t adcAccumulators[ADC_MAX_CHANNELS];
static uint8_t adcAccumulatorCounters[ADC_MAX_CHANNELS];

static uint16_t adcErrorLevels[ADC_MAX_CHANNELS];
static uint8_t adcErrorCounters[ADC_MAX_CHANNELS];

int adcReadingToVoltage(int raw)
{
    int voltage = 0;
    if (adc_cali_raw_to_voltage(adcCaliHandle, raw, &voltage) != ESP_OK)
    {
        // CORE_WARN("Calibrated voltage reading failed! Using linear estimation");
        voltage = (raw * 3100UL) / ((1UL << 12) - 1);
    }
    return (int32_t)voltage;
}

int pinToIdx(gpio_num_t gpio)
{
    adc_unit_t unit;
    adc_channel_t channel = ADC_CHANNEL_0;
    adc_continuous_io_to_channel(gpio, &unit, &channel);
    return channel;
}

bool isPinValidAdc(gpio_num_t gpio, const char* errMsg = nullptr, adc_channel_t* outChannel = nullptr)
{
    adc_unit_t unit;
    adc_channel_t channel;
    if ((adc_continuous_io_to_channel(gpio, &unit, &channel) != ESP_OK) && (unit != ADC_UNIT_1))
    {
        if (errMsg)
            CORE_ERROR(errMsg, gpio);
        return false;
    }

    if (outChannel)
        *outChannel = channel;

    return true;
}

bool adcConvCompleteCallback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata, void* user_data)
{
    const uint8_t resultCount = (edata->size / 4);
    for (int i = 0; i < resultCount; i++)
    {
        const adc_digi_output_data_t result = ((adc_digi_output_data_t*)edata->conv_frame_buffer)[i];
        const uint16_t vtg = adcReadingToVoltage(result.type2.data);
        const int idx = resultIndices[i];

        if (vtg > adcErrorLevels[idx])
        {
            adcErrorCounters[idx]++;
        }
        else
        {
            adcAccumulators[idx] += vtg;
        }

        adcAccumulatorCounters[idx]++;
        if (adcAccumulatorCounters[idx] >= ADC_OVERSAMPLE_COUNT)
        {
            if (adcErrorCounters[idx] > ADC_ERROR_SAMPLE_COUNT)
                adcVoltages[idx].store(ROVER_ADC_ERR_RETURN_VAL);
            else
                adcVoltages[idx].store(adcAccumulators[idx] / adcAccumulatorCounters[idx]);

            hasNewData[idx].store(true);

            adcAccumulatorCounters[idx] = 0;
            adcAccumulators[idx] = 0;
            adcErrorCounters[idx] = 0;
        }
    }
    return false;
}

void adcDeinit()
{
    if (!initialised)
    {
        CORE_ERROR("ADC not initialised, cannot de-init!");
        return;
    }
    CORE_DEBUG("De-initialising ADC");

    adc_continuous_stop(adcHandle);
    adc_continuous_deinit(adcHandle);
    adc_cali_delete_scheme_curve_fitting(adcCaliHandle);

    memset(adcAccumulators, 0, sizeof(adcAccumulators));
    memset(adcAccumulatorCounters, 0, sizeof(adcAccumulatorCounters));
    for (int i = 0; i < ADC_MAX_CHANNELS; i++)
    {
        hasNewData[i].store(false);
        adcVoltages[i].store(0);
    }

    initialised = false;
    CORE_DEBUG("ADC de-initialised");
}

void adcSetErrorVoltage(gpio_num_t gpio, uint16_t errorVtg)
{
    if (!isPinValidAdc(gpio, "Error setting pin %d error voltage - is not valid ADC channel!"))
        return;

    int idx = pinToIdx(gpio);
    CORE_DEBUG("ADC channel %d error voltage now %u", idx, errorVtg);
    adcErrorLevels[idx] = errorVtg;
}
void adcSetChannelEnabled(gpio_num_t gpio, bool enable, bool delayedInit)
{
    static bool isInitialised[ADC_MAX_CHANNELS]{};

    adc_channel_t channel;
    if (!isPinValidAdc(gpio, "Error configuring pin %d - is not valid ADC channel!", &channel))
        return;

    CORE_DEBUG("%s ADC channel %d, pin %d", enable ? "Enabling" : "Disabling", channel, gpio);

    int idx = pinToIdx(gpio);
    isInitialised[idx] = enable;
    adcAccumulatorCounters[idx] = 0;
    adcAccumulators[idx] = 0;
    adcErrorCounters[idx] = 0;
    adcErrorLevels[idx] = ROVER_ADC_DISABLE_ERROR;
    if (delayedInit)
        return;

    adc_digi_pattern_config_t adcConversionConfig[ADC_MAX_CHANNELS]{};
    resultIndexCount = 0;
    for (int i = 0; i < ADC_MAX_CHANNELS; i++)
    {
        if (isInitialised[i])
        {
            adcConversionConfig[resultIndexCount] = {
                .atten = ADC_ATTEN_DB_12,
                .channel = (uint8_t)i,
                .unit = ADC_UNIT_1,
                .bit_width = 12,
            };
            resultIndices[resultIndexCount] = i;
            resultIndexCount++;
        }
    }

    if (initialised)
        adcDeinit();

    if (resultIndexCount == 0)
    {
        CORE_DEBUG("No channels enabled!");
        return;  // if no channels enabled, nothing to do!
    }

    adc_cali_curve_fitting_config_t adcCalibrationConfig = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_0,  // not used! provided for extensibility as of ESP-IDF v5.1.1
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    // non-returning err check since we can do a linear estimation even if calibrated conversion fails
    ROVER_CORE_ERR_CHECK("ADC calibration scheme creation failed!",
                         adc_cali_create_scheme_curve_fitting(&adcCalibrationConfig, &adcCaliHandle));

    CORE_DEBUG("%d ADC channels enabled - configuring...", resultIndexCount);
    adc_continuous_handle_cfg_t adcConfig = {
        .max_store_buf_size = 4U * resultIndexCount,
        .conv_frame_size = 4U * resultIndexCount,
        .flags = {
            .flush_pool = true,
        },
    };
    ROVER_CORE_RET_ERR_CHECK("ADC initialisation failed!", adc_continuous_new_handle(&adcConfig, &adcHandle));

    adc_continuous_config_t adcChanConfig = {
        .pattern_num = resultIndexCount,
        .adc_pattern = adcConversionConfig,
        .sample_freq_hz = 1000 * ADC_OVERSAMPLE_COUNT,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,     // we only use unit 1
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,  // only type2 is valid
    };
    ROVER_CORE_RET_ERR_CHECK("Error configuring ADC!", adc_continuous_config(adcHandle, &adcChanConfig));

    adc_continuous_evt_cbs_t adcCallbacks = {
        .on_conv_done = adcConvCompleteCallback,
        .on_pool_ovf = NULL,
    };
    ROVER_CORE_RET_ERR_CHECK("Error registering ADC callbacks!",
                             adc_continuous_register_event_callbacks(adcHandle, &adcCallbacks, NULL));
    ROVER_CORE_RET_ERR_CHECK("Error starting ADC!", adc_continuous_start(adcHandle));

    initialised = true;

    CORE_DEBUG("ADC initialisation complete");
}

bool adcHasNewReading(gpio_num_t gpio)
{
    if (!isPinValidAdc(gpio))
        return false;
    return hasNewData[pinToIdx(gpio)].load();
}
int32_t adcGetVoltage(gpio_num_t gpio)
{
    if (!isPinValidAdc(gpio, "Attempting to read ADC value for un-managed pin %d!"))
        return 0;

    const int idx = pinToIdx(gpio);
    hasNewData[idx].store(false);
    return adcVoltages[idx].load();
}