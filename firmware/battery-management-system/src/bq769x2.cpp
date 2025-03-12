#include "bq769x2.hpp"

#include <Wire.h>
#include <crc.h>

#include <thread>
#include <type.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

bq76942::bq76942(uint8_t address) : _address(address)
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
    const auto start = steady_clock::now();
    // exact timeout doesn't matter, it shouldn't ever take long
    while ((steady_clock::now() - start) < 100ms)
    {
        if (readBatteryStatus().inConfigUpdateMode)
            return;
        std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("Failed to enter config update mode - timed out!");
}

void bq76942::setGPO(const gpo& pin, const bool state)
{
    switch (pin)
    {
    default:
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
    std::this_thread::sleep_for(10ms);
    writeSubcommand(cmd_only_subcommand::PF_FORCE_B);
}

bq76942::firmware_version_t bq76942::readFirmwareVersion()
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

bq76942::security_keys_t bq76942::readSecurityKeys()
{
    security_keys_t rawKeys = readSubcommand<security_keys_t>(subcommand::SECURITY_KEYS);
    // keys are stored in big-endian format so wew need to re-construct them manually
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

void bq76942::writeSecurityKeys(const security_keys_t& keys)
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

bq76942::manufacturer_data_t bq76942::readManufacturerData()
{
    const auto data = readSubcommand(subcommand::MANU_DATA);
    manufacturer_data_t manuData;
    if (data.size() != manuData.size())
        throw std::runtime_error("Manufacturer data size mismatch");
    std::copy(data.begin(), data.end(), manuData.begin());
    return manuData;
}

std::vector<uint8_t> bq76942::readDirect(const uint8_t registerAddr, const size_t bytes)
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

void bq76942::writeDirect(const uint8_t registerAddr, const std::vector<uint8_t>& data)
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

std::vector<uint8_t> bq76942::readSubcommand(const uint16_t registerAddr)
{
    writeDirect(direct_command::SUBCOMMAND, registerAddr);
    // most subcommands take between 400-660us to complete,
    // so if we wait 1ms now we won't have to poll more than once
    // in most cases
    std::this_thread::sleep_for(1ms);

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

    // read data length + checksum
    std::vector<uint8_t> dataLenChecksum = readDirect(direct_command::TRANSFER_CHECKSUM, 2);
    if (dataLenChecksum.size() != 2)
    {
        throw std::runtime_error("Failed to read data length and checksum");
    }

    const uint8_t expectedChecksum = dataLenChecksum[0];
    // data len includes itself, the checksum, and the subcommand,
    // so subtract 4 for the transfer buffer data len
    const uint8_t dataLen = dataLenChecksum[1] - 4;

    printf("Data len: %d\n", dataLen);
    std::vector<uint8_t> data = readDirect(direct_command::TRANSFER_BUF, dataLen);
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
void bq76942::writeSubcommand(const uint16_t registerAddr, const std::vector<uint8_t>& data)
{
    if (data.size() > 32)
    {
        throw std::runtime_error("Data size too large");
    }
    writeDirect(direct_command::SUBCOMMAND, registerAddr);
    // command-only subcommand (no data)
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

uint8_t bq76942::_calculateChecksum(uint16_t registerAddr, const std::vector<uint8_t>& data) const
{
    uint8_t checksum = 0;
    // checksum includes the subcommand as well as the data
    checksum += static_cast<uint8_t>(registerAddr & 0xFF);
    checksum += static_cast<uint8_t>((registerAddr >> 8) & 0xFF);
    for (const uint8_t byte : data)
        checksum += byte;
    return ~checksum;
}

int16_t bq76942::readCellVoltage(const uint8_t cell)
{
    if (cell > 16)
    {
        throw std::invalid_argument("Invalid cell number");
    }
    int8_t address = static_cast<uint8_t>(direct_command::CELL_1_VOLTAGE) + 2 * cell;
    return readDirect<int16_t>(address);
}

int32_t bq76942::readStackVoltage()
{
    const da_configuration_t daConfig = settings.configuration.readDAConfiguration();
    int32_t stackVoltageInUserVolts = readDirect<int16_t>(direct_command::STACK_VOLTAGE);
    if (daConfig.userVoltsIsCentivolts)
        return stackVoltageInUserVolts * 10;
    return stackVoltageInUserVolts;
}

int32_t bq76942::readPackVoltage()
{
    const da_configuration_t daConfig = settings.configuration.readDAConfiguration();
    int32_t packVoltageInUserVolts = readDirect<int16_t>(direct_command::PACK_PIN_VOLTAGE);
    if (daConfig.userVoltsIsCentivolts)
        return packVoltageInUserVolts * 10;
    return packVoltageInUserVolts;
}

int32_t bq76942::readLdVoltage()
{
    const da_configuration_t daConfig = settings.configuration.readDAConfiguration();
    int32_t ldVoltageInUserVolts = readDirect<int16_t>(direct_command::LD_PIN_VOLTAGE);
    if (daConfig.userVoltsIsCentivolts)
        return ldVoltageInUserVolts * 10;
    return ldVoltageInUserVolts;
}

float bq76942::readCC2Current()
{
    const da_configuration_t daConfig = settings.configuration.readDAConfiguration();
    int32_t cc2CurrentInUserAmps = readDirect<int16_t>(direct_command::CC2_CURRENT);
    switch (daConfig.userAmps)
    {
    default:
        return cc2CurrentInUserAmps;
    case user_amps::DECIMILLIAMP:
        return cc2CurrentInUserAmps / 10.0f;
    case user_amps::MILLIAMP:
        return cc2CurrentInUserAmps;
    case user_amps::CENTIAMP:
        return cc2CurrentInUserAmps * 10;
    case user_amps::DECIAMP:
        return cc2CurrentInUserAmps * 100;
    }
}