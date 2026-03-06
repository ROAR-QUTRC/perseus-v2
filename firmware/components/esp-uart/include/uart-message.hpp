#pragma once
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

typedef std::vector<uint8_t> raw_uart_message_t;

struct basic_uart_message_t
{
    basic_uart_message_t() = default;

    /// Makes any struct that inherits from this struct able to override this operator to be used as a raw_uart_message_t. The override can't change the struct's data (const), and this implementation returns 0, making it purely virtual
    virtual operator raw_uart_message_t() const = 0;
};

constexpr uint8_t DATA_LENGTH_BYTES = 1;
constexpr uint8_t CRC_BYTES = 1;

/// STRUCTURE of sent data:
/// header (however many bytes)
/// data_length (one byte)
/// data (however many bytes)
/// trailer (however many bytes)
/// crc (one byte)
/// footer (however many bytes)
struct flagged_uart_message_t : public basic_uart_message_t
{
    std::vector<uint8_t> header = {};
    uint8_t _data_length = 0;
    raw_uart_message_t data = {};
    std::vector<uint8_t> trailer = {};

    typedef std::optional<std::function<std::vector<uint8_t>(const raw_uart_message_t)>> crc_creator_t;
    crc_creator_t crc_creator = std::nullopt;

    std::vector<uint8_t> footer = {};

    flagged_uart_message_t(std::vector<uint8_t> _header, raw_uart_message_t _data, std::vector<uint8_t> _trailer, crc_creator_t _crc_creator, std::vector<uint8_t> _footer)
        : header(_header),
          data(_data),
          trailer(_trailer),
          crc_creator(_crc_creator),
          footer(_footer)
    {
    }

    explicit operator raw_uart_message_t() const override
    {
        std::vector<uint8_t> raw_message = {};
        raw_message.insert(raw_message.end(), header.begin(), header.end());
        raw_message.emplace_back(data.size());
        raw_message.insert(raw_message.end(), data.begin(), data.end());
        raw_message.insert(raw_message.end(), trailer.begin(), trailer.end());
        if (crc_creator.has_value())
        {
            std::vector<uint8_t> crc = crc_creator.value()(data);
            raw_message.insert(raw_message.end(), crc.begin(), crc.end());
        }
        raw_message.insert(raw_message.end(), footer.begin(), footer.end());
        return raw_message;
    }
};
