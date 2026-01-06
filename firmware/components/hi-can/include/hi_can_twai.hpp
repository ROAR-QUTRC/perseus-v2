#pragma once

#include <driver/twai.h>

#include <board_support.hpp>
#include <chrono>
#include <cstdint>
#include <hi_can.hpp>
#include <optional>
#include <tuple>

namespace hi_can
{
    class TwaiInterface : public FilteredCanInterface
    {
    public:
        static TwaiInterface& get_instance(bsp::pin_pair_t pins = std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), uint8_t controller_id = 0, addressing::filter_t filter = {});
        virtual ~TwaiInterface();

        // allow moving but not copying
        TwaiInterface(const TwaiInterface&) = delete;
        TwaiInterface(TwaiInterface&& other) noexcept
            : TwaiInterface()
        {
            swap(*this, other);
        }
        TwaiInterface& operator=(const TwaiInterface&) = delete;
        TwaiInterface& operator=(TwaiInterface&& other) noexcept
        {
            swap(*this, other);
            return *this;
        }

        void transmit(const Packet& packet) override;
        std::optional<Packet> receive(bool blocking = false) override;

        /**
         * @brief Handle error detection and recovery on the underlying TWAI bus
         *
         * This function must be called regularly to ensure that the bus recovers and has errors handled correctly.
         *
         */
        void handle();

        TwaiInterface& add_filter(const addressing::filter_t& address) override;
        TwaiInterface& remove_filter(const addressing::filter_t& address) override;

        // swap function for move semantics
        friend void swap(TwaiInterface& first, TwaiInterface& second) noexcept
        {
            using std::swap;
            swap(first._controller_id, second._controller_id);
            swap(first._twai_bus, second._twai_bus);
            swap(first._received_packets, second._received_packets);
        }

    private:
        static constexpr uint8_t INVALID_INTERFACE_ID = 255;
        TwaiInterface() = default;  // FOR MOVE SEMANTICS ONLY
        TwaiInterface(bsp::pin_pair_t pins, uint8_t controller_id, addressing::filter_t filter);

        uint8_t _controller_id = INVALID_INTERFACE_ID;
        twai_handle_t _twai_bus;

        uint _recovery_attempt_count = 0;

        std::vector<Packet> _received_packets;
    };
}