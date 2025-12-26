#include "bq769x2.hpp"

#include <Wire.h>
#include <crc.h>

#include <thread>
#include <type_demangle.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

bq76942::bq76942(uint8_t address)
    : _address(address)
{
    Wire.beginTransmission(_address);
    if (Wire.endTransmission())
    {
        throw std::runtime_error("Device not found");
    }
    // maximum data ever in CRC buffer is 4 bytes:
    // One for the address, one for the register, one for the re-started address, and one for the data byte
    _data_buf.reserve(4);
}

void bq76942::enter_deep_sleep()
{
    write_subcommand(cmd_only_subcommand::DEEPSLEEP);
    // see comment in shutdown() - must be written twice within 4s
    std::this_thread::sleep_for(100ms);
    write_subcommand(cmd_only_subcommand::DEEPSLEEP);
}

void bq76942::shutdown()
{
    write_subcommand(bq76942::cmd_only_subcommand::ALL_FETS_OFF);
    write_subcommand(cmd_only_subcommand::SHUTDOWN);
    // needs to be written twice within 4s to apply
    // NOTE: This doesn't apply in UNSEALED or FULLACCESS modes,
    // and the IC will shut down immediately in these modes.
    std::this_thread::sleep_for(100ms);
    write_subcommand(cmd_only_subcommand::SHUTDOWN);
}

void bq76942::enter_config_update_mode()
{
    write_subcommand(cmd_only_subcommand::SET_CFGUPDATE);
    // wait for IC to enter config update mode
    // TRM says 2ms approximate delay,
    // so may as well wait out the guaranteed time before checking
    std::this_thread::sleep_for(2.5ms);
    const auto start = steady_clock::now();
    // exact timeout doesn't matter, it shouldn't ever take long
    while ((steady_clock::now() - start) < 10ms)
    {
        if (get_battery_status().in_config_update_mode)
            return;
        std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("Failed to enter config update mode - timed out!");
}

void bq76942::exit_config_update_mode()
{
    write_subcommand(cmd_only_subcommand::EXIT_CFGUPDATE);
    // wait for command to finish
    std::this_thread::sleep_for(2ms);
}

void bq76942::set_GPO(const gpo& pin, const bool state)
{
    switch (pin)
    {
    default:
        throw std::invalid_argument("Invalid GPO pin");
        break;
    case gpo::CFETOFF:
        write_subcommand(state ? cmd_only_subcommand::CFETOFF_HI : cmd_only_subcommand::CFETOFF_LO);
        break;
    case gpo::DFETOFF:
        write_subcommand(state ? cmd_only_subcommand::DFETOFF_HI : cmd_only_subcommand::DFETOFF_LO);
        break;
    case gpo::ALERT:
        write_subcommand(state ? cmd_only_subcommand::ALERT_HI : cmd_only_subcommand::ALERT_LO);
        break;
    case gpo::HDQ:
        write_subcommand(state ? cmd_only_subcommand::HDQ_HI : cmd_only_subcommand::HDQ_LO);
        break;
    case gpo::DCHG:
        write_subcommand(state ? cmd_only_subcommand::DCHG_HI : cmd_only_subcommand::DCHG_LO);
        break;
    case gpo::DDSG:
        write_subcommand(state ? cmd_only_subcommand::DDSG_HI : cmd_only_subcommand::DDSG_LO);
        break;
    };
}

void bq76942::force_permanent_fail()
{
    // to force permanent fail, word A and B must be written in order within 4s
    write_subcommand(cmd_only_subcommand::PF_FORCE_A);
    std::this_thread::sleep_for(100ms);
    write_subcommand(cmd_only_subcommand::PF_FORCE_B);
}

bq76942::firmware_version_t bq76942::get_firmware_version()
{
    firmware_version_t raw_version = read_subcommand<firmware_version_t>(subcommand::FW_VERSION);
    uint16_t device_number = (raw_version.device_number_bytes[0] << 8) | raw_version.device_number_bytes[1];
    // note: version number may be BCD too? not entirely clear in the TRM. Not a big deal though.
    uint16_t version_number = (raw_version.firmware_version_bytes[0] << 8) | raw_version.firmware_version_bytes[1];
    uint16_t firmware_version = (raw_version.bcd_high_of_high_byte * 1000) +
                                (raw_version.bcd_high_of_low_byte * 100) +
                                (raw_version.bcd_low_of_high_byte * 10) +
                                raw_version.bcd_low_of_low_byte;
    return {device_number, version_number, firmware_version};
}

bq76942::security_keys_t bq76942::get_security_keys()
{
    security_keys_t raw_keys = read_subcommand<security_keys_t>(subcommand::SECURITY_KEYS);
    // keys are stored in big-endian format so we need to re-construct them manually
    return security_keys_t{
        .unseal = {
            .step_1 = static_cast<uint16_t>((raw_keys.unseal.step_1_bytes[0] << 8) | raw_keys.unseal.step_1_bytes[1]),
            .step_2 = static_cast<uint16_t>((raw_keys.unseal.step_2_bytes[0] << 8) | raw_keys.unseal.step_2_bytes[1]),
        },
        .full_access = {
            .step_1 = static_cast<uint16_t>((raw_keys.full_access.step_1_bytes[0] << 8) | raw_keys.full_access.step_1_bytes[1]),
            .step_2 = static_cast<uint16_t>((raw_keys.full_access.step_2_bytes[0] << 8) | raw_keys.full_access.step_2_bytes[1]),
        },
    };
}

void bq76942::set_security_keys(const security_keys_t& keys)
{
    const security_keys_t big_endian_keys{
        .unseal = {
            .step_1_bytes = {static_cast<uint8_t>((keys.unseal.step_1 >> 8) & 0xFF), static_cast<uint8_t>(keys.unseal.step_1 & 0xFF)},
            .step_2_bytes = {static_cast<uint8_t>((keys.unseal.step_2 >> 8) & 0xFF), static_cast<uint8_t>(keys.unseal.step_2 & 0xFF)},
        },
        .full_access = {
            .step_1_bytes = {static_cast<uint8_t>((keys.full_access.step_1 >> 8) & 0xFF), static_cast<uint8_t>(keys.full_access.step_1 & 0xFF)},
            .step_2_bytes = {static_cast<uint8_t>((keys.full_access.step_2 >> 8) & 0xFF), static_cast<uint8_t>(keys.full_access.step_2 & 0xFF)},
        },
    };
    write_subcommand(subcommand::SECURITY_KEYS, big_endian_keys);
}

bq76942::manufacturer_data_t bq76942::get_manufacturer_data()
{
    const auto data_vec = read_subcommand(subcommand::MANU_DATA);
    manufacturer_data_t data_arr;
    if (data_vec.size() != data_arr.size())
        throw std::runtime_error("Manufacturer data size mismatch");
    std::copy(data_vec.begin(), data_vec.end(), data_arr.begin());
    return data_arr;
}

void bq76942::set_manufacturer_data(const manufacturer_data_t& data_arr)
{
    std::vector<uint8_t> data_vec(data_arr.size(), 0);
    std::copy(data_arr.begin(), data_arr.end(), data_vec.begin());
    write_subcommand(subcommand::MANU_DATA, data_vec);
}

bq76942::da_status_5_t bq76942::get_DA_status_5()
{
    const raw_da_status_5_t raw_status = get_raw_DA_status_5();
    const da_configuration_t da_config = settings.configuration.get_DA_configuration();
    return da_status_5_t{
        .vreg_18_adc_counts = raw_status.vreg_18_adc_counts,
        .vss_adc_counts = raw_status.vss_adc_counts,
        .max_cell_voltage = raw_status.max_cell_voltage,
        .min_cell_voltage = raw_status.min_cell_voltage,
        .battery_voltage_sum = raw_status.battery_voltage_sum * get_user_volts_multiplier(da_config),
        .cell_temperature = raw_temp_to_celsius(raw_status.cell_temperature),
        .fet_temperature = raw_temp_to_celsius(raw_status.fet_temperature),
        .max_cell_temperature = raw_temp_to_celsius(raw_status.max_cell_temperature),
        .min_cell_temperature = raw_temp_to_celsius(raw_status.min_cell_temperature),
        .avg_cell_temperature = raw_temp_to_celsius(raw_status.avg_cell_temperature),
        .cc_3_current = static_cast<float>(raw_status.cc_3_current) * get_user_amps_multiplier(da_config),
        .cc_1_current = static_cast<float>(raw_status.cc_1_current) * get_user_amps_multiplier(da_config),
        .cc_2_counts = raw_status.cc_2_counts,
        .cc_3_counts = raw_status.cc_3_counts,
    };
}

bq76942::da_status_6_t bq76942::get_DA_status_6()
{
    const raw_da_status_6_t raw_status = get_raw_DA_status_6();
    const da_configuration_t da_config = settings.configuration.get_DA_configuration();

    const float acc_charge_integer = static_cast<float>(raw_status.accumulated_charge);
    float acc_charge_fractional = static_cast<float>(raw_status.accumulated_charge_fraction) / (1ULL << 32);
    acc_charge_fractional -= 0.5;  // starts initialised to +0.5 user_ah, zero that out

    return da_status_6_t{
        .accumulated_charge = (acc_charge_integer + acc_charge_fractional) * get_user_amps_multiplier(da_config),
        .accumulated_charge_time = std::chrono::seconds(raw_status.accumulated_charge_time),
        .cfetoff_counts = raw_status.cfetoff_counts,
        .dfetoff_counts = raw_status.dfetoff_counts,
        .alert_counts = raw_status.alert_counts,
        .ts1_counts = raw_status.ts1_counts,
        .ts2_counts = raw_status.ts2_counts,
    };
}

std::vector<uint8_t> bq76942::read_direct(const uint8_t register_addr, const size_t bytes)
{
    for (int i = 0; i <= MAX_RETRIES; i++)
    {
        try
        {
            return _read_direct(register_addr, bytes);
        }
        catch (...)
        {
            if (i >= MAX_RETRIES)
                throw;
            std::this_thread::sleep_for(1ms);
        }
    }
    // control can never reach here (the loop will either return or throw),
    // but let's make the compiler happy
    return {};
}

void bq76942::write_direct(const uint8_t register_addr, const std::vector<uint8_t>& data)
{
    for (int i = 0; i <= MAX_RETRIES; i++)
    {
        try
        {
            _write_direct(register_addr, data);
            return;
        }
        catch (...)
        {
            if (i >= MAX_RETRIES)
                throw;
            // TODO: Change this to be not a magic constant
            std::this_thread::sleep_for(10ms);
        }
    }
}

std::vector<uint8_t> bq76942::read_subcommand(const uint16_t register_addr)
{
    for (int i = 0; i <= MAX_RETRIES; i++)
    {
        try
        {
            return _read_subcommand(register_addr);
        }
        catch (...)
        {
            if (i >= MAX_RETRIES)
                throw;
            std::this_thread::sleep_for(10ms);
        }
    }
    // control can never reach here (the loop will either return or throw),
    // but let's make the compiler happy
    return {};
}

void bq76942::write_subcommand(const uint16_t register_addr, const std::vector<uint8_t>& data)
{
    // no need for retries here since it should be handled by the underlying direct_write calls
    _write_subcommand(register_addr, data);
}

float bq76942::get_user_amps_multiplier(const da_configuration_t& config)
{
    switch (config.user_amps)
    {
    default:
        return 1.0f;
        break;
    case user_amps_unit::DECIMILLIAMP:
        return 0.1f;
        break;
    case user_amps_unit::MILLIAMP:
        return 1.0f;
        break;
    case user_amps_unit::CENTIAMP:
        return 10.0f;
        break;
    case user_amps_unit::DECIAMP:
        return 100.0f;
        break;
    }
}

int32_t bq76942::get_stack_voltage()
{
    return read_direct<int16_t>(direct_command::STACK_VOLTAGE) * get_user_volts_multiplier(settings.configuration.get_DA_configuration());
}

int32_t bq76942::get_pack_voltage()
{
    return read_direct<int16_t>(direct_command::PACK_PIN_VOLTAGE) * get_user_volts_multiplier(settings.configuration.get_DA_configuration());
}

int32_t bq76942::get_ld_voltage()
{
    return read_direct<int16_t>(direct_command::LD_PIN_VOLTAGE) * get_user_volts_multiplier(settings.configuration.get_DA_configuration());
}

float bq76942::get_CC2_current()
{
    return read_direct<int16_t>(direct_command::CC2_CURRENT) * get_user_amps_multiplier(settings.configuration.get_DA_configuration());
}

int16_t bq76942::Calibration::Voltage::get_cell_gain(const uint8_t& cell) const
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int16_t register_addr = static_cast<uint16_t>(bq76942::data_register::CELL_1_GAIN) + (2 * cell);
    return _parent.read_subcommand<int16_t>(register_addr);
}

void bq76942::Calibration::Voltage::set_cell_gain(const uint8_t& cell, const int16_t& gain) const
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int16_t register_addr = static_cast<uint16_t>(bq76942::data_register::CELL_1_GAIN) + (2 * cell);
    _parent.write_direct(register_addr, gain);
}

uint32_t bq76942::Settings::get_dsg_current_threshold() const
{
    return _parent.read_subcommand<int16_t>(data_register::DSG_CURRENT_THRESHOLD) * get_user_amps_multiplier(configuration.get_DA_configuration());
}

void bq76942::Settings::set_dsg_current_threshold(const uint32_t& threshold) const
{
    const int16_t rounded_threshold = static_cast<int16_t>(std::round(threshold / _parent.get_user_amps_multiplier(_parent.settings.configuration.get_DA_configuration())));
    _parent.write_subcommand_clamped<int16_t>(data_register::DSG_CURRENT_THRESHOLD, rounded_threshold, 0, std::numeric_limits<int16_t>::max());
}

uint32_t bq76942::Settings::get_chg_current_threshold() const
{
    return _parent.read_subcommand<int16_t>(data_register::CHG_CURRENT_THRESHOLD) * get_user_amps_multiplier(configuration.get_DA_configuration());
}

void bq76942::Settings::set_chg_current_threshold(const uint32_t& threshold) const
{
    const int16_t rounded_threshold = static_cast<int16_t>(std::round(threshold / _parent.get_user_amps_multiplier(_parent.settings.configuration.get_DA_configuration())));
    _parent.write_subcommand_clamped<int16_t>(data_register::CHG_CURRENT_THRESHOLD, rounded_threshold, 0, std::numeric_limits<int16_t>::max());
}

int16_t bq76942::Settings::get_cell_interconnect_resistance(const uint8_t& cell)
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int16_t register_addr = static_cast<uint16_t>(bq76942::data_register::CELL_1_INTERCONNECT) + (2 * cell);
    return _parent.read_direct<int16_t>(register_addr);
}

void bq76942::Settings::set_cell_interconnect_resistance(const uint8_t& cell, const int16_t& resistance)
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int16_t register_addr = static_cast<uint16_t>(bq76942::data_register::CELL_1_INTERCONNECT) + (2 * cell);
    _parent.write_direct(register_addr, resistance);
}

float bq76942::Protections::OverCurrentDischarge::get_tier_3_threshold() const
{
    return _parent.read_subcommand<int16_t>(data_register::OCD3_THRESHOLD) * get_user_amps_multiplier(_parent.settings.configuration.get_DA_configuration());
}

void bq76942::Protections::OverCurrentDischarge::set_tier_3_threshold(const float& threshold) const
{
    const int16_t rounded_threshold = static_cast<int16_t>(std::round(threshold / _parent.get_user_amps_multiplier(_parent.settings.configuration.get_DA_configuration())));
    _parent.write_subcommand_clamped<int16_t>(data_register::OCD3_THRESHOLD, rounded_threshold, std::numeric_limits<int16_t>::min(), 0);
}

float bq76942::Protections::get_precharge_reset_charge() const
{
    return _parent.read_subcommand<int16_t>(data_register::PTO_RESET) * get_user_amps_multiplier(_parent.settings.configuration.get_DA_configuration());
}

void bq76942::Protections::set_precharge_reset_charge(const float& charge) const
{
    const int16_t rounded_charge = static_cast<int16_t>(std::round(charge / _parent.get_user_amps_multiplier(_parent.settings.configuration.get_DA_configuration())));
    _parent.write_subcommand_clamped<int16_t>(data_register::PTO_RESET, rounded_charge, 0, 10000);
}

float bq76942::PermanentFail::get_charge_current_threshold() const
{
    const da_configuration_t da_config = _parent.settings.configuration.get_DA_configuration();
    return _parent.read_subcommand<int16_t>(data_register::SOCC_THRESHOLD) * get_user_amps_multiplier(da_config);
}

void bq76942::PermanentFail::set_charge_current_threshold(const float& threshold) const
{
    const da_configuration_t da_config = _parent.settings.configuration.get_DA_configuration();
    const int16_t rounded_threshold = static_cast<int16_t>(std::round(threshold / get_user_amps_multiplier(da_config)));
    _parent.write_subcommand(data_register::SOCC_THRESHOLD, rounded_threshold);
}

float bq76942::PermanentFail::get_discharge_current_threshold() const
{
    const da_configuration_t da_config = _parent.settings.configuration.get_DA_configuration();
    return _parent.read_subcommand<int16_t>(data_register::SOCD_THRESHOLD) * get_user_amps_multiplier(da_config);
}

void bq76942::PermanentFail::set_discharge_current_threshold(const float& threshold) const
{
    const da_configuration_t da_config = _parent.settings.configuration.get_DA_configuration();
    const int16_t rounded_threshold = static_cast<int16_t>(std::round(threshold / get_user_amps_multiplier(da_config)));
    _parent.write_subcommand(data_register::SOCD_THRESHOLD, rounded_threshold);
}

int16_t bq76942::get_user_volts_multiplier(const da_configuration_t& config)
{
    if (config.user_volts_is_centivolts)
        return 10;
    return 1;
}

int16_t bq76942::get_cell_voltage(const uint8_t cell)
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int8_t address = static_cast<uint8_t>(direct_command::CELL_1_VOLTAGE) + (2 * cell);
    return read_direct<int16_t>(address);
}

std::vector<uint8_t> bq76942::_read_direct(const uint8_t register_addr, const size_t bytes)
{
    if (bytes == 0)
    {
        throw std::invalid_argument("Cannot read 0 bytes");
    }
    Wire.beginTransmission(_address);
    Wire.write(register_addr);
    if (Wire.endTransmission(false))
    {
        throw std::runtime_error("Failed to select register");
    }

    // each data byte has a CRC
    const size_t real_byte_count = bytes * 2;
    const size_t bytes_read = Wire.requestFrom(_address, real_byte_count);

    _data_buf.clear();
    // preload data buffer with address and register
    // for first CRC calculation
    _data_buf.push_back(_address << 1);
    _data_buf.push_back(register_addr);
    // OR 1 because I2C read is LSB set
    _data_buf.push_back((_address << 1) | 0x01);

    // printf(std::format("Read {} bytes\n", bytes_read).c_str());
    if (bytes_read != real_byte_count)
    {
        throw std::runtime_error(std::format("Read failed: Expected {} bytes, got {}", real_byte_count, bytes_read));
    }

    std::vector<uint8_t> real_data_buf;
    real_data_buf.reserve(bytes);
    while (Wire.available())
    {
        uint8_t data = static_cast<uint8_t>(Wire.read());
        uint8_t crc = static_cast<uint8_t>(Wire.read());
        _data_buf.push_back(data);
        real_data_buf.push_back(data);
        if (crc != crc8(_data_buf.data(), _data_buf.size()))
        {
            throw std::runtime_error(std::format("CRC mismatch at data byte {:#04x}", real_data_buf.size()));
        }
        // print data in hex
        // printf(std::format("{:#04x} ", data).c_str());
        _data_buf.clear();
    }
    // printf("\n");
    return real_data_buf;
}

void bq76942::_write_direct(const uint8_t register_addr, const std::vector<uint8_t>& data)
{
    Wire.beginTransmission(_address);
    Wire.write(register_addr);

    // same CRC calculation as in read_register
    _data_buf.clear();
    _data_buf.push_back(_address << 1);
    _data_buf.push_back(register_addr);

    for (const uint8_t byte : data)
    {
        _data_buf.push_back(byte);
        uint8_t crc = crc8(_data_buf.data(), _data_buf.size());
        Wire.write(byte);
        Wire.write(crc);
        _data_buf.clear();
    }
    if (uint8_t err = Wire.endTransmission(); err)
    {
        throw std::runtime_error(std::format("Failed to write data with error {}", err));
    }
}

void bq76942::_select_subcommand(const uint16_t register_addr, bool wait_for_completion)
{
    write_direct(direct_command::SUBCOMMAND, register_addr);

    if (!wait_for_completion)
        return;
    // most subcommands take between 400-660us to complete,
    // so if we wait 1ms now we won't have to poll more than once
    // in most cases
    std::this_thread::sleep_for(660us);

    steady_clock::time_point start = steady_clock::now();
    bool command_success = false;
    while ((steady_clock::now() - start) < SUBCOMMAND_TIMEOUT)
    {
        uint16_t command_readback = read_direct<uint16_t>(direct_command::SUBCOMMAND);
        if (command_readback == register_addr)
        {
            command_success = true;
            break;
        }
        std::this_thread::sleep_for(1ms);
    }
    if (!command_success)
    {
        throw std::runtime_error(std::format("Subcommand {:#06x} timeout", register_addr));
    }
}

std::vector<uint8_t> bq76942::_read_subcommand(const uint16_t register_addr)
{
    _select_subcommand(register_addr);

    // read data length, the data, and the checksum (in that order - reading the checksum before data causes issues)
    // data len includes itself, the checksum, and the subcommand,
    // so subtract 4 for the transfer buffer data len
    const auto data_len = read_direct<uint8_t>(direct_command::TRANSFER_LEN) - 4;
    const auto data = read_direct(direct_command::TRANSFER_BUF, data_len);
    const auto expected_checksum = read_direct<uint8_t>(direct_command::TRANSFER_CHECKSUM);
    if (data.size() != data_len)
    {
        throw std::runtime_error(std::format("Data size mismatch - got {} bytes, expected {}",
                                             data.size(), data_len));
    }

    // calculate checksum
    const uint8_t real_checksum = _calculate_checksum(register_addr, data);
    if (real_checksum != expected_checksum)
    {
        throw std::runtime_error(std::format("Checksum mismatch - got {:#04x}, expected {:#04x}",
                                             real_checksum, expected_checksum));
    }
    return data;
}

void bq76942::_write_subcommand(const uint16_t register_addr, const std::vector<uint8_t>& data)
{
    if (data.size() > 32)
    {
        throw std::runtime_error("Data size too large");
    }
    _select_subcommand(register_addr, false);
    // command only subcommand
    if (data.size() == 0)
        return;

    // write data
    write_direct(direct_command::TRANSFER_BUF, data);
    const uint8_t checksum = _calculate_checksum(register_addr, data);
    // plus 4 for the subcommand address (2 bytes), data length, and checksum
    const uint8_t data_len = static_cast<uint8_t>(data.size() + 4);
    std::vector<uint8_t> data_len_checksum = {checksum, data_len};
    write_direct(direct_command::TRANSFER_CHECKSUM, data_len_checksum);
}

uint8_t bq76942::_calculate_checksum(uint16_t register_addr, const std::vector<uint8_t>& data)
{
    uint8_t checksum = 0;
    // checksum includes the subcommand as well as the data
    checksum += static_cast<uint8_t>(register_addr & 0xFF);
    checksum += static_cast<uint8_t>((register_addr >> 8) & 0xFF);
    for (const uint8_t byte : data)
        checksum += byte;
    return ~checksum;
}

void bq76942::Calibration::Current::set_sense_resistor_value(const float& value) const
{
    const float cc_gain = 7.4768 / value;
    const float capacity_gain = cc_gain * 298261.6178;
    set_CC_gain(cc_gain);
    set_capacity_gain(capacity_gain);
}
