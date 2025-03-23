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
    _dataBuf.reserve(4);
}

void bq76942::enterDeepSleep()
{
    writeSubcommand(cmd_only_subcommand::DEEPSLEEP);
    // see comment in shutdown() - must be written twice within 4s
    std::this_thread::sleep_for(100ms);
    writeSubcommand(cmd_only_subcommand::DEEPSLEEP);
}

void bq76942::shutdown()
{
    writeSubcommand(bq76942::cmd_only_subcommand::ALL_FETS_OFF);
    writeSubcommand(cmd_only_subcommand::SHUTDOWN);
    // needs to be written twice within 4s to apply
    // NOTE: This doesn't apply in UNSEALED or FULLACCESS modes,
    // and the IC will shut down immediately in these modes.
    std::this_thread::sleep_for(100ms);
    writeSubcommand(cmd_only_subcommand::SHUTDOWN);
}

void bq76942::enterConfigUpdateMode()
{
    writeSubcommand(cmd_only_subcommand::SET_CFGUPDATE);
    // wait for IC to enter config update mode
    // TRM says 2ms approximate delay,
    // so may as well wait out the guaranteed time before checking
    std::this_thread::sleep_for(2.5ms);
    const auto start = steady_clock::now();
    // exact timeout doesn't matter, it shouldn't ever take long
    while ((steady_clock::now() - start) < 10ms)
    {
        if (getBatteryStatus().inConfigUpdateMode)
            return;
        std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("Failed to enter config update mode - timed out!");
}

void bq76942::exitConfigUpdateMode()
{
    writeSubcommand(cmd_only_subcommand::EXIT_CFGUPDATE);
    // wait for command to finish
    std::this_thread::sleep_for(2ms);
}

void bq76942::setGPO(const gpo& pin, const bool state)
{
    switch (pin)
    {
    default:
        throw std::invalid_argument("Invalid GPO pin");
        break;
    case gpo::CFETOFF:
        writeSubcommand(state ? cmd_only_subcommand::CFETOFF_HI : cmd_only_subcommand::CFETOFF_LO);
        break;
    case gpo::DFETOFF:
        writeSubcommand(state ? cmd_only_subcommand::DFETOFF_HI : cmd_only_subcommand::DFETOFF_LO);
        break;
    case gpo::ALERT:
        writeSubcommand(state ? cmd_only_subcommand::ALERT_HI : cmd_only_subcommand::ALERT_LO);
        break;
    case gpo::HDQ:
        writeSubcommand(state ? cmd_only_subcommand::HDQ_HI : cmd_only_subcommand::HDQ_LO);
        break;
    case gpo::DCHG:
        writeSubcommand(state ? cmd_only_subcommand::DCHG_HI : cmd_only_subcommand::DCHG_LO);
        break;
    case gpo::DDSG:
        writeSubcommand(state ? cmd_only_subcommand::DDSG_HI : cmd_only_subcommand::DDSG_LO);
        break;
    };
}

void bq76942::forcePermanentFail()
{
    // to force permanent fail, word A and B must be written in order within 4s
    writeSubcommand(cmd_only_subcommand::PF_FORCE_A);
    std::this_thread::sleep_for(100ms);
    writeSubcommand(cmd_only_subcommand::PF_FORCE_B);
}

bq76942::firmware_version_t bq76942::getFirmwareVersion()
{
    firmware_version_t rawVersion = readSubcommand<firmware_version_t>(subcommand::FW_VERSION);
    uint16_t deviceNumber = (rawVersion.deviceNumberBytes[0] << 8) | rawVersion.deviceNumberBytes[1];
    // note: version number may be BCD too? not entirely clear in the TRM. Not a big deal though.
    uint16_t versionNumber = (rawVersion.firmwareVersionBytes[0] << 8) | rawVersion.firmwareVersionBytes[1];
    uint16_t firmwareVersion = (rawVersion.bcdHighOfHighByte * 1000) +
                               (rawVersion.bcdHighOfLowByte * 100) +
                               (rawVersion.bcdLowOfHighByte * 10) +
                               rawVersion.bcdLowOfLowByte;
    return {deviceNumber, versionNumber, firmwareVersion};
}

bq76942::security_keys_t bq76942::getSecurityKeys()
{
    security_keys_t rawKeys = readSubcommand<security_keys_t>(subcommand::SECURITY_KEYS);
    // keys are stored in big-endian format so we need to re-construct them manually
    return security_keys_t{
        .unseal = {
            .step1 = static_cast<uint16_t>((rawKeys.unseal.step1Bytes[0] << 8) | rawKeys.unseal.step1Bytes[1]),
            .step2 = static_cast<uint16_t>((rawKeys.unseal.step2Bytes[0] << 8) | rawKeys.unseal.step2Bytes[1]),
        },
        .fullAccess = {
            .step1 = static_cast<uint16_t>((rawKeys.fullAccess.step1Bytes[0] << 8) | rawKeys.fullAccess.step1Bytes[1]),
            .step2 = static_cast<uint16_t>((rawKeys.fullAccess.step2Bytes[0] << 8) | rawKeys.fullAccess.step2Bytes[1]),
        },
    };
}

void bq76942::setSecurityKeys(const security_keys_t& keys)
{
    const security_keys_t bigEndianKeys{
        .unseal = {
            .step1Bytes = {static_cast<uint8_t>((keys.unseal.step1 >> 8) & 0xFF), static_cast<uint8_t>(keys.unseal.step1 & 0xFF)},
            .step2Bytes = {static_cast<uint8_t>((keys.unseal.step2 >> 8) & 0xFF), static_cast<uint8_t>(keys.unseal.step2 & 0xFF)},
        },
        .fullAccess = {
            .step1Bytes = {static_cast<uint8_t>((keys.fullAccess.step1 >> 8) & 0xFF), static_cast<uint8_t>(keys.fullAccess.step1 & 0xFF)},
            .step2Bytes = {static_cast<uint8_t>((keys.fullAccess.step2 >> 8) & 0xFF), static_cast<uint8_t>(keys.fullAccess.step2 & 0xFF)},
        },
    };
    writeSubcommand(subcommand::SECURITY_KEYS, bigEndianKeys);
}

bq76942::manufacturer_data_t bq76942::getManufacturerData()
{
    const auto dataVec = readSubcommand(subcommand::MANU_DATA);
    manufacturer_data_t dataArr;
    if (dataVec.size() != dataArr.size())
        throw std::runtime_error("Manufacturer data size mismatch");
    std::copy(dataVec.begin(), dataVec.end(), dataArr.begin());
    return dataArr;
}

void bq76942::setManufacturerData(const manufacturer_data_t& dataArr)
{
    std::vector<uint8_t> dataVec(dataArr.size(), 0);
    std::copy(dataArr.begin(), dataArr.end(), dataVec.begin());
    writeSubcommand(subcommand::MANU_DATA, dataVec);
}

bq76942::da_status_5_t bq76942::getDAStatus5()
{
    const raw_da_status_5_t rawStatus = getRawDAStatus5();
    const da_configuration_t daConfig = settings.configuration.getDAConfiguration();
    return da_status_5_t{
        .vreg18AdcCounts = rawStatus.vreg18AdcCounts,
        .vssAdcCounts = rawStatus.vssAdcCounts,
        .maxCellVoltage = rawStatus.maxCellVoltage,
        .minCellVoltage = rawStatus.minCellVoltage,
        .batteryVoltageSum = rawStatus.batteryVoltageSum * getUserVoltsMultiplier(daConfig),
        .cellTemperature = rawTempToCelsius(rawStatus.cellTemperature),
        .fetTemperature = rawTempToCelsius(rawStatus.fetTemperature),
        .maxCellTemperature = rawTempToCelsius(rawStatus.maxCellTemperature),
        .minCellTemperature = rawTempToCelsius(rawStatus.minCellTemperature),
        .avgCellTemperature = rawTempToCelsius(rawStatus.avgCellTemperature),
        .cc3Current = static_cast<float>(rawStatus.cc3Current) * getUserAmpsMultiplier(daConfig),
        .cc1Current = static_cast<float>(rawStatus.cc1Current) * getUserAmpsMultiplier(daConfig),
        .cc2Counts = rawStatus.cc2Counts,
        .cc3Counts = rawStatus.cc3Counts,
    };
}

bq76942::da_status_6_t bq76942::getDAStatus6()
{
    const raw_da_status_6_t rawStatus = getRawDAStatus6();
    const da_configuration_t daConfig = settings.configuration.getDAConfiguration();

    const float accChargeInteger = static_cast<float>(rawStatus.accumulatedCharge);
    const float accChargeFractional = static_cast<float>(rawStatus.accumulatedChargeFraction) / (1ULL << 32);

    return da_status_6_t{
        .accumulatedCharge = (accChargeInteger + accChargeFractional) * getUserAmpsMultiplier(daConfig),
        .accumulatedChargeTime = std::chrono::seconds(rawStatus.accumulatedChargeTime),
        .cfetoffCounts = rawStatus.cfetoffCounts,
        .dfetoffCounts = rawStatus.dfetoffCounts,
        .alertCounts = rawStatus.alertCounts,
        .ts1Counts = rawStatus.ts1Counts,
        .ts2Counts = rawStatus.ts2Counts,
    };
}

std::vector<uint8_t> bq76942::readDirect(const uint8_t registerAddr, const size_t bytes)
{
    for (int i = 0; i <= MAX_RETRIES; i++)
    {
        try
        {
            return _readDirect(registerAddr, bytes);
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

void bq76942::writeDirect(const uint8_t registerAddr, const std::vector<uint8_t>& data)
{
    for (int i = 0; i <= MAX_RETRIES; i++)
    {
        try
        {
            _writeDirect(registerAddr, data);
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

std::vector<uint8_t> bq76942::readSubcommand(const uint16_t registerAddr)
{
    for (int i = 0; i <= MAX_RETRIES; i++)
    {
        try
        {
            return _readSubcommand(registerAddr);
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

void bq76942::writeSubcommand(const uint16_t registerAddr, const std::vector<uint8_t>& data)
{
    // no need for retries here since it should be handled by the underlying directWrite calls
    _writeSubcommand(registerAddr, data);
}

float bq76942::getUserAmpsMultiplier(const da_configuration_t& config)
{
    switch (config.userAmps)
    {
    default:
        return 1.0f;
        break;
    case user_amps::DECIMILLIAMP:
        return 0.1f;
        break;
    case user_amps::MILLIAMP:
        return 1.0f;
        break;
    case user_amps::CENTIAMP:
        return 10.0f;
        break;
    case user_amps::DECIAMP:
        return 100.0f;
        break;
    }
}

int32_t bq76942::getStackVoltage()
{
    return readDirect<int16_t>(direct_command::STACK_VOLTAGE) * getUserVoltsMultiplier(settings.configuration.getDAConfiguration());
}

int32_t bq76942::getPackVoltage()
{
    return readDirect<int16_t>(direct_command::PACK_PIN_VOLTAGE) * getUserVoltsMultiplier(settings.configuration.getDAConfiguration());
}

int32_t bq76942::getLdVoltage()
{
    return readDirect<int16_t>(direct_command::LD_PIN_VOLTAGE) * getUserVoltsMultiplier(settings.configuration.getDAConfiguration());
}

float bq76942::getCC2Current()
{
    return readDirect<int16_t>(direct_command::CC2_CURRENT) * getUserAmpsMultiplier(settings.configuration.getDAConfiguration());
}

int16_t bq76942::Calibration::Voltage::getCellGain(const uint8_t& cell) const
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int16_t registerAddr = static_cast<uint16_t>(bq76942::data_register::CELL_1_GAIN) + (2 * cell);
    return _parent.readSubcommand<int16_t>(registerAddr);
}

void bq76942::Calibration::Voltage::setCellGain(const uint8_t& cell, const int16_t& gain) const
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int16_t registerAddr = static_cast<uint16_t>(bq76942::data_register::CELL_1_GAIN) + (2 * cell);
    _parent.writeDirect(registerAddr, gain);
}

uint32_t bq76942::Settings::getDsgCurrentThreshold() const
{
    return _parent.readSubcommand<int16_t>(data_register::DSG_CURRENT_THRESHOLD) * getUserAmpsMultiplier(configuration.getDAConfiguration());
}

void bq76942::Settings::setDsgCurrentThreshold(const uint32_t& threshold) const
{
    const int16_t roundedThreshold = static_cast<int16_t>(std::round(threshold / _parent.getUserAmpsMultiplier(_parent.settings.configuration.getDAConfiguration())));
    _parent.writeSubcommandClamped<int16_t>(data_register::DSG_CURRENT_THRESHOLD, roundedThreshold, 0, std::numeric_limits<int16_t>::max());
}

uint32_t bq76942::Settings::getChgCurrentThreshold() const
{
    return _parent.readSubcommand<int16_t>(data_register::CHG_CURRENT_THRESHOLD) * getUserAmpsMultiplier(configuration.getDAConfiguration());
}

void bq76942::Settings::setChgCurrentThreshold(const uint32_t& threshold) const
{
    const int16_t roundedThreshold = static_cast<int16_t>(std::round(threshold / _parent.getUserAmpsMultiplier(_parent.settings.configuration.getDAConfiguration())));
    _parent.writeSubcommandClamped<int16_t>(data_register::CHG_CURRENT_THRESHOLD, roundedThreshold, 0, std::numeric_limits<int16_t>::max());
}

int16_t bq76942::Settings::getCellInterconnectResistance(const uint8_t& cell)
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int16_t registerAddr = static_cast<uint16_t>(bq76942::data_register::CELL_1_INTERCONNECT) + (2 * cell);
    return _parent.readDirect<int16_t>(registerAddr);
}

void bq76942::Settings::setCellInterconnectResistance(const uint8_t& cell, const int16_t& resistance)
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int16_t registerAddr = static_cast<uint16_t>(bq76942::data_register::CELL_1_INTERCONNECT) + (2 * cell);
    _parent.writeDirect(registerAddr, resistance);
}

float bq76942::Protections::OverCurrentDischarge::getTier3Threshold() const
{
    return _parent.readSubcommand<int16_t>(data_register::OCD3_THRESHOLD) * getUserAmpsMultiplier(_parent.settings.configuration.getDAConfiguration());
}

void bq76942::Protections::OverCurrentDischarge::setTier3Threshold(const float& threshold) const
{
    const int16_t roundedThreshold = static_cast<int16_t>(std::round(threshold / _parent.getUserAmpsMultiplier(_parent.settings.configuration.getDAConfiguration())));
    _parent.writeSubcommandClamped<int16_t>(data_register::OCD3_THRESHOLD, roundedThreshold, std::numeric_limits<int16_t>::min(), 0);
}

float bq76942::Protections::getPrechargeResetCharge() const
{
    return _parent.readSubcommand<int16_t>(data_register::PTO_RESET) * getUserAmpsMultiplier(_parent.settings.configuration.getDAConfiguration());
}

void bq76942::Protections::setPrechargeResetCharge(const float& charge) const
{
    const int16_t roundedCharge = static_cast<int16_t>(std::round(charge / _parent.getUserAmpsMultiplier(_parent.settings.configuration.getDAConfiguration())));
    _parent.writeSubcommandClamped<int16_t>(data_register::PTO_RESET, roundedCharge, 0, 10000);
}

float bq76942::PermanentFail::getChargeCurrentThreshold() const
{
    const da_configuration_t daConfig = _parent.settings.configuration.getDAConfiguration();
    return _parent.readSubcommand<int16_t>(data_register::SOCC_THRESHOLD) * getUserAmpsMultiplier(daConfig);
}

void bq76942::PermanentFail::setChargeCurrentThreshold(const float& threshold) const
{
    const da_configuration_t daConfig = _parent.settings.configuration.getDAConfiguration();
    const int16_t roundedThreshold = static_cast<int16_t>(std::round(threshold / getUserAmpsMultiplier(daConfig)));
    _parent.writeSubcommand(data_register::SOCC_THRESHOLD, roundedThreshold);
}

float bq76942::PermanentFail::getDischargeCurrentThreshold() const
{
    const da_configuration_t daConfig = _parent.settings.configuration.getDAConfiguration();
    return _parent.readSubcommand<int16_t>(data_register::SOCD_THRESHOLD) * getUserAmpsMultiplier(daConfig);
}

void bq76942::PermanentFail::setDischargeCurrentThreshold(const float& threshold) const
{
    const da_configuration_t daConfig = _parent.settings.configuration.getDAConfiguration();
    const int16_t roundedThreshold = static_cast<int16_t>(std::round(threshold / getUserAmpsMultiplier(daConfig)));
    _parent.writeSubcommand(data_register::SOCD_THRESHOLD, roundedThreshold);
}

int16_t bq76942::getUserVoltsMultiplier(const da_configuration_t& config)
{
    if (config.userVoltsIsCentivolts)
        return 10;
    return 1;
}

int16_t bq76942::getCellVoltage(const uint8_t cell)
{
    if (cell >= 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    const int8_t address = static_cast<uint8_t>(direct_command::CELL_1_VOLTAGE) + (2 * cell);
    return readDirect<int16_t>(address);
}

std::vector<uint8_t> bq76942::_readDirect(const uint8_t registerAddr, const size_t bytes)
{
    if (bytes == 0)
    {
        throw std::invalid_argument("Cannot read 0 bytes");
    }
    Wire.beginTransmission(_address);
    Wire.write(registerAddr);
    if (Wire.endTransmission(false))
    {
        throw std::runtime_error("Failed to select register");
    }

    // each data byte has a CRC
    const size_t realByteCount = bytes * 2;
    const size_t bytesRead = Wire.requestFrom(_address, realByteCount);

    _dataBuf.clear();
    // preload data buffer with address and register
    // for first CRC calculation
    _dataBuf.push_back(_address << 1);
    _dataBuf.push_back(registerAddr);
    // OR 1 because I2C read is LSB set
    _dataBuf.push_back((_address << 1) | 0x01);

    // printf(std::format("Read {} bytes\n", bytesRead).c_str());
    if (bytesRead != realByteCount)
    {
        throw std::runtime_error(std::format("Read failed: Expected {} bytes, got {}", realByteCount, bytesRead));
    }

    std::vector<uint8_t> realDataBuf;
    realDataBuf.reserve(bytes);
    while (Wire.available())
    {
        uint8_t data = static_cast<uint8_t>(Wire.read());
        uint8_t crc = static_cast<uint8_t>(Wire.read());
        _dataBuf.push_back(data);
        realDataBuf.push_back(data);
        if (crc != crc8(_dataBuf.data(), _dataBuf.size()))
        {
            throw std::runtime_error(std::format("CRC mismatch at data byte {:#04x}", realDataBuf.size()));
        }
        // print data in hex
        // printf(std::format("{:#04x} ", data).c_str());
        _dataBuf.clear();
    }
    // printf("\n");
    return realDataBuf;
}

void bq76942::_writeDirect(const uint8_t registerAddr, const std::vector<uint8_t>& data)
{
    Wire.beginTransmission(_address);
    Wire.write(registerAddr);

    // same CRC calculation as in readRegister
    _dataBuf.clear();
    _dataBuf.push_back(_address << 1);
    _dataBuf.push_back(registerAddr);

    for (const uint8_t byte : data)
    {
        _dataBuf.push_back(byte);
        uint8_t crc = crc8(_dataBuf.data(), _dataBuf.size());
        Wire.write(byte);
        Wire.write(crc);
        _dataBuf.clear();
    }
    if (uint8_t err = Wire.endTransmission(); err)
    {
        throw std::runtime_error(std::format("Failed to write data with error {}", err));
    }
}

void bq76942::_selectSubcommand(const uint16_t registerAddr, bool waitForCompletion)
{
    writeDirect(direct_command::SUBCOMMAND, registerAddr);

    if (!waitForCompletion)
        return;
    // most subcommands take between 400-660us to complete,
    // so if we wait 1ms now we won't have to poll more than once
    // in most cases
    std::this_thread::sleep_for(660us);

    steady_clock::time_point start = steady_clock::now();
    bool commandSuccess = false;
    while ((steady_clock::now() - start) < SUBCOMMAND_TIMEOUT)
    {
        uint16_t commandReadback = readDirect<uint16_t>(direct_command::SUBCOMMAND);
        if (commandReadback == registerAddr)
        {
            commandSuccess = true;
            break;
        }
        std::this_thread::sleep_for(1ms);
    }
    if (!commandSuccess)
    {
        throw std::runtime_error(std::format("Subcommand {:#06x} timeout", registerAddr));
    }
}

std::vector<uint8_t> bq76942::_readSubcommand(const uint16_t registerAddr)
{
    _selectSubcommand(registerAddr);

    // read data length, the data, and the checksum (in that order - reading the checksum before data causes issues)
    // data len includes itself, the checksum, and the subcommand,
    // so subtract 4 for the transfer buffer data len
    const auto dataLen = readDirect<uint8_t>(direct_command::TRANSFER_LEN) - 4;
    const auto data = readDirect(direct_command::TRANSFER_BUF, dataLen);
    const auto expectedChecksum = readDirect<uint8_t>(direct_command::TRANSFER_CHECKSUM);
    if (data.size() != dataLen)
    {
        throw std::runtime_error(std::format("Data size mismatch - got {} bytes, expected {}",
                                             data.size(), dataLen));
    }

    // calculate checksum
    const uint8_t realChecksum = _calculateChecksum(registerAddr, data);
    if (realChecksum != expectedChecksum)
    {
        throw std::runtime_error(std::format("Checksum mismatch - got {:#04x}, expected {:#04x}",
                                             realChecksum, expectedChecksum));
    }
    return data;
}

void bq76942::_writeSubcommand(const uint16_t registerAddr, const std::vector<uint8_t>& data)
{
    if (data.size() > 32)
    {
        throw std::runtime_error("Data size too large");
    }
    _selectSubcommand(registerAddr, false);
    // command only subcommand
    if (data.size() == 0)
        return;

    // write data
    writeDirect(direct_command::TRANSFER_BUF, data);
    const uint8_t checksum = _calculateChecksum(registerAddr, data);
    // plus 4 for the subcommand address (2 bytes), data length, and checksum
    const uint8_t dataLen = static_cast<uint8_t>(data.size() + 4);
    std::vector<uint8_t> dataLenChecksum = {checksum, dataLen};
    writeDirect(direct_command::TRANSFER_CHECKSUM, dataLenChecksum);
}

uint8_t bq76942::_calculateChecksum(uint16_t registerAddr, const std::vector<uint8_t>& data)
{
    uint8_t checksum = 0;
    // checksum includes the subcommand as well as the data
    checksum += static_cast<uint8_t>(registerAddr & 0xFF);
    checksum += static_cast<uint8_t>((registerAddr >> 8) & 0xFF);
    for (const uint8_t byte : data)
        checksum += byte;
    return ~checksum;
}

void bq76942::Calibration::Current::setSenseResistorValue(const float& value) const
{
    const float ccGain = 7.4768 / value;
    const float capacityGain = ccGain * 298261.6178;
    setCCGain(ccGain);
    setCapacityGain(capacityGain);
}
