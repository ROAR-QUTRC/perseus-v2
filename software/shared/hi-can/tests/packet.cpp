#include <gtest/gtest.h>
#include <netinet/in.h>

#include "hi_can_address.hpp"
#include "hi_can_packet.hpp"

using namespace hi_can;
using namespace addressing;

#pragma pack(push, 1)
struct test_data_t
{
    uint8_t a;
    uint8_t b;
    uint16_t c;
    uint32_t d;
};
#pragma pack(pop)

TEST(Packet, DefaultConstructor)
{
    Packet packet;
    std::vector<uint8_t> expectedData{};
    EXPECT_EQ(static_cast<raw_address_t>(packet.getAddress()), MAX_ADDRESS);
    EXPECT_EQ(packet.getData(), expectedData);
    EXPECT_EQ(packet.getDataLen(), expectedData.size());
}

TEST(Packet, AddressConstructor)
{
    raw_address_t address = 0x12345678;
    Packet packet{flagged_address_t{address}};
    std::vector<uint8_t> expectedData{};
    EXPECT_EQ(static_cast<raw_address_t>(packet.getAddress()), address);
    EXPECT_EQ(packet.getData(), expectedData);
}

TEST(Packet, FullConstructor)
{
    raw_address_t address = 0x12345678;
    std::vector<uint8_t> data{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    Packet packet{flagged_address_t{address}, data};
    EXPECT_EQ(static_cast<raw_address_t>(packet.getAddress()), address);
    EXPECT_EQ(packet.getData(), data);
}

TEST(Packet, TemplateConstructor)
{
    raw_address_t address = 0x12345678;

    test_data_t data{0x01, 0x02, htons(0x0304), htonl(0x05060708)};
    Packet packet{flagged_address_t{address}, data};
    std::vector<uint8_t> expectedData{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    EXPECT_EQ(static_cast<raw_address_t>(packet.getAddress()), address);
    EXPECT_EQ(packet.getData(), expectedData);
}

TEST(Packet, RawDataConstructor)
{
    raw_address_t address = 0x12345678;
    std::vector<uint8_t> data{0x01, 0x02, 0x03, 0x04};
    Packet packet{flagged_address_t{address}, data.data(), data.size()};
    EXPECT_EQ(static_cast<raw_address_t>(packet.getAddress()), address);
    EXPECT_EQ(packet.getData(), data);
}

TEST(Packet, InvalidConstructors)
{
    raw_address_t address = MAX_ADDRESS;
    std::vector<uint8_t> data(MAX_PACKET_LEN + 1);
    EXPECT_THROW(Packet(flagged_address_t{address}, data), std::invalid_argument);
    EXPECT_THROW(Packet(flagged_address_t{address}, data.data(), data.size()), std::invalid_argument);
    EXPECT_THROW(Packet(flagged_address_t{address}, nullptr, data.size()), std::invalid_argument);
    flagged_address_t invalidAddress{address};
    invalidAddress.address = 0xFFFFFFFF;
    EXPECT_THROW(Packet{invalidAddress}, std::invalid_argument);

    struct
    {
        uint8_t a;
        uint8_t b;
        uint16_t c;
        uint32_t d;
        uint32_t e;
    } invalidData;
    test_data_t validData;
    EXPECT_THROW(Packet(flagged_address_t{address}, invalidData), std::invalid_argument);
    EXPECT_THROW(Packet(invalidAddress, validData), std::invalid_argument);
}

TEST(Packet, RawDataGetters)
{
    raw_address_t address = 0x12345678;
    std::vector<uint8_t> data{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    Packet packet{flagged_address_t{address}, data};
    EXPECT_EQ(packet.getData(), data);
    EXPECT_EQ(packet.getDataLen(), data.size());
}
TEST(Packet, RawDataSetters)
{
    std::vector<uint8_t> data{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    Packet packet;
    packet.setData(data);
    EXPECT_EQ(packet.getData(), data);
}

TEST(Packet, TemplateDataGetter)
{
    std::vector<uint8_t> data{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    Packet packet{flagged_address_t{}, data};

    auto optionalData = packet.getData<test_data_t>();
    ASSERT_TRUE(optionalData.has_value());

    test_data_t actualData = optionalData.value();
    EXPECT_EQ(actualData.a, data[0]);
    EXPECT_EQ(actualData.b, data[1]);
    // no need to convert these using ntoh*,
    // because the data in the vector is already in network byte order
    uint16_t c;
    std::copy_n(data.begin() + 2, sizeof(c), reinterpret_cast<uint8_t*>(&c));
    uint32_t d;
    std::copy_n(data.begin() + 4, sizeof(d), reinterpret_cast<uint8_t*>(&d));
    EXPECT_EQ(actualData.c, c);
    EXPECT_EQ(actualData.d, d);
}
TEST(Packet, InvalidTemplateDataGetter)
{
    std::vector<uint8_t> data{0x01, 0x02, 0x03, 0x04};
    Packet packet{flagged_address_t{}, data};

    auto optionalData = packet.getData<test_data_t>();
    EXPECT_FALSE(optionalData.has_value());
}
TEST(Packet, TemplateDataSetter)
{
    Packet packet{};
    test_data_t data{0x01, 0x02, htons(0x0304), htonl(0x05060708)};
    std::vector<uint8_t> expectedData{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    ASSERT_NO_THROW(packet.setData(data));

    auto optionalData = packet.getData<test_data_t>();
    ASSERT_TRUE(optionalData.has_value());

    test_data_t actualData = optionalData.value();
    EXPECT_EQ(actualData.a, data.a);
    EXPECT_EQ(actualData.b, data.b);
    EXPECT_EQ(actualData.c, data.c);
    EXPECT_EQ(actualData.d, data.d);
}

TEST(Packet, AddressGetters)
{
    flagged_address_t noFlagsAddress{MAX_SHORT_ADDRESS, false, false, false};
    Packet packetNoFlags{noFlagsAddress};
    EXPECT_EQ(packetNoFlags.getAddress(), noFlagsAddress);
    EXPECT_EQ(packetNoFlags.getIsExtended(), noFlagsAddress.isExtended);
    EXPECT_EQ(packetNoFlags.getIsRTR(), noFlagsAddress.isRtr);
    EXPECT_EQ(packetNoFlags.getIsError(), noFlagsAddress.isError);

    flagged_address_t allFlagsAddress{MAX_ADDRESS, true, true, true};
    Packet packetAllFlags{allFlagsAddress};
    EXPECT_EQ(packetAllFlags.getAddress(), allFlagsAddress);
    EXPECT_EQ(packetAllFlags.getIsExtended(), allFlagsAddress.isExtended);
    EXPECT_EQ(packetAllFlags.getIsRTR(), allFlagsAddress.isRtr);
    EXPECT_EQ(packetAllFlags.getIsError(), allFlagsAddress.isError);
}
TEST(Packet, AddressSetters)
{
    flagged_address_t address{MAX_SHORT_ADDRESS, false, false, false};
    Packet packet;
    packet.setAddress(address);
    EXPECT_EQ(packet.getAddress(), address);

    flagged_address_t newAddress{MAX_ADDRESS, true, true, true};
    packet.setAddress(newAddress);
    EXPECT_EQ(packet.getAddress(), newAddress);
}

TEST(Packet, AddressFlagsSetters)
{
    flagged_address_t address{MAX_SHORT_ADDRESS, false, false, false};
    Packet packet;
    packet.setIsRTR(true);
    packet.setIsError(true);
    packet.setIsExtended(true);
    EXPECT_EQ(packet.getIsRTR(), true);
    EXPECT_EQ(packet.getIsError(), true);
    EXPECT_EQ(packet.getIsExtended(), true);

    packet.setIsRTR(false);
    packet.setIsError(false);
    packet.setIsExtended(false);
    EXPECT_EQ(packet.getIsRTR(), false);
    EXPECT_EQ(packet.getIsError(), false);
    EXPECT_EQ(packet.getIsExtended(), false);
}

TEST(Packet, AddressEquality)
{
    flagged_address_t address1{0x123, false, false, false};
    flagged_address_t address2{0x123, false, false, false};
    flagged_address_t address3{0x123, true, false, false};
    flagged_address_t address4{0x123, false, true, false};
    flagged_address_t address5{0x12345678, false, false, true};

    Packet packet1{address1};
    Packet packet2{address2};
    Packet packet3{address3};
    Packet packet4{address4};
    Packet packet5{address5};

    EXPECT_EQ(packet1, packet2);
    EXPECT_NE(packet1, packet3);
    EXPECT_NE(packet1, packet4);
    EXPECT_NE(packet1, packet5);
}

TEST(Packet, DataEquality)
{
    std::vector<uint8_t> data1{0x01, 0x02, 0x03, 0x04};
    std::vector<uint8_t> data2{0x01, 0x02, 0x03, 0x04};
    std::vector<uint8_t> data3{0x01, 0x02, 0x03, 0x05};
    std::vector<uint8_t> data4{0x01, 0x02, 0x03, 0x04, 0x05};

    Packet packet1{flagged_address_t{}, data1};
    Packet packet2{flagged_address_t{}, data2};
    Packet packet3{flagged_address_t{}, data3};
    Packet packet4{flagged_address_t{}, data4};

    EXPECT_EQ(packet1, packet2);
    EXPECT_NE(packet1, packet3);
    EXPECT_NE(packet1, packet4);
}

TEST(Packet, CombinedEquality)
{
    flagged_address_t address1{0x12345678, false, false, true};
    flagged_address_t address2{0x12345678, false, false, true};
    flagged_address_t address3{0x12345678, true, false, true};
    flagged_address_t address4{0x12345678, false, true, true};

    std::vector<uint8_t> data1{0x01, 0x02, 0x03, 0x04};
    std::vector<uint8_t> data2{0x01, 0x02, 0x03, 0x04};
    std::vector<uint8_t> data3{0x01, 0x02, 0x03, 0x05};
    std::vector<uint8_t> data4{0x01, 0x02, 0x03, 0x04, 0x05};

    Packet packet1{address1, data1};
    Packet packet2{address2, data2};
    Packet packet3{address3, data3};
    Packet packet4{address4, data4};
    Packet packet5{address1, data3};
    Packet packet6{address3, data1};

    EXPECT_EQ(packet1, packet2);
    EXPECT_NE(packet1, packet3);
    EXPECT_NE(packet1, packet4);
    EXPECT_NE(packet1, packet5);
    EXPECT_NE(packet1, packet6);
}

TEST(Packet, Comparison)
{
    // note: internally, Packet only compares by address
    // This allows sorting in STL containers
    // equality is implemented (and tested) separately
    flagged_address_t address1{0x12345678, false, false, true};
    flagged_address_t address2{0x12345678, false, false, true};
    flagged_address_t address3{0x1FFFFFFF, false, false, true};
    flagged_address_t address4{0x12345678, true, false, true};
    flagged_address_t address5{0x12345678, false, true, true};
    flagged_address_t address6{0x123, false, false, false};
    flagged_address_t address7{0x123, false, false, true};

    Packet packet1{address1};
    Packet packet2{address2};
    Packet packet3{address3};
    Packet packet4{address4};
    Packet packet5{address5};
    Packet packet6{address6};
    Packet packet7{address7};

    EXPECT_FALSE(packet1 < packet2);
    EXPECT_FALSE(packet1 > packet2);
    EXPECT_FALSE(packet2 < packet1);
    EXPECT_FALSE(packet2 > packet1);
    EXPECT_GE(packet1, packet2);
    EXPECT_LE(packet1, packet2);
    EXPECT_GE(packet2, packet1);
    EXPECT_LE(packet2, packet1);

    EXPECT_LT(packet1, packet3);
    EXPECT_GT(packet3, packet1);
    EXPECT_LE(packet1, packet3);
    EXPECT_GE(packet3, packet1);

    EXPECT_LT(packet1, packet4);
    EXPECT_GT(packet4, packet1);
    EXPECT_LE(packet1, packet4);
    EXPECT_GE(packet4, packet1);

    EXPECT_LT(packet1, packet5);
    EXPECT_GT(packet5, packet1);
    EXPECT_LE(packet1, packet5);
    EXPECT_GE(packet5, packet1);

    EXPECT_GT(packet1, packet6);
    EXPECT_LT(packet6, packet1);
    EXPECT_GE(packet1, packet6);
    EXPECT_LE(packet6, packet1);

    EXPECT_LT(packet6, packet7);
    EXPECT_GT(packet7, packet6);
    EXPECT_LE(packet6, packet7);
    EXPECT_GE(packet7, packet6);
}