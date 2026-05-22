// Shared helpers for hi-can unit tests.
//
// Defines big-endian byte builders and a concat for assembling golden CAN
// payloads. Always source golden bytes from the device datasheet (or from
// hand-computed scaling factors), not by round-tripping through the code
// under test — otherwise a bug in the serializer hides itself.

#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>

namespace hi_can_test
{
    inline std::vector<uint8_t> be32(int32_t v)
    {
        // Cast to unsigned so the right-shifts are unambiguously logical,
        // not implementation-defined arithmetic on signed integers.
        const uint32_t u = static_cast<uint32_t>(v);
        return {
            static_cast<uint8_t>((u >> 24) & 0xFFu),
            static_cast<uint8_t>((u >> 16) & 0xFFu),
            static_cast<uint8_t>((u >> 8) & 0xFFu),
            static_cast<uint8_t>(u & 0xFFu),
        };
    }

    inline std::vector<uint8_t> be16(int16_t v)
    {
        const uint16_t u = static_cast<uint16_t>(v);
        return {
            static_cast<uint8_t>((u >> 8) & 0xFFu),
            static_cast<uint8_t>(u & 0xFFu),
        };
    }

    inline std::vector<uint8_t> concat(std::initializer_list<std::vector<uint8_t>> parts)
    {
        std::vector<uint8_t> out;
        for (const auto& p : parts)
            out.insert(out.end(), p.begin(), p.end());
        return out;
    }
}  // namespace hi_can_test
