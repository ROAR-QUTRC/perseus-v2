#include <gtest/gtest.h>

#include <cstdint>

#include "hi_can_address.hpp"

using namespace hi_can::addressing;

TEST(StandardAddress, DefaultConstructor)
{
    standard_address_t address;
    EXPECT_EQ(static_cast<raw_address_t>(address), 0);
    EXPECT_EQ(address.system, 0);
    EXPECT_EQ(address.subsystem, 0);
    EXPECT_EQ(address.device, 0);
    EXPECT_EQ(address.group, 0);
    EXPECT_EQ(address.parameter, 0);
}

TEST(StandardAddress, FullConstructor)
{
    standard_address_t address(0x01, 0x02, 0x03, 0x04, 0x05);
    EXPECT_EQ(static_cast<raw_address_t>(address), 0x00A30405);
    EXPECT_EQ(address.system, 0x01);
    EXPECT_EQ(address.subsystem, 0x02);
    EXPECT_EQ(address.device, 0x03);
    EXPECT_EQ(address.group, 0x04);
    EXPECT_EQ(address.parameter, 0x05);
}

TEST(StandardAddress, Clipping)
{
    standard_address_t paramTest(0, 0, 0, 0, 0xFF);
    EXPECT_EQ(static_cast<raw_address_t>(paramTest), 0x000000FF);
    EXPECT_EQ(paramTest.parameter, 0xFF);

    standard_address_t groupTest(0, 0, 0, 0xFF, 0);
    EXPECT_EQ(static_cast<raw_address_t>(groupTest), 0x0000FF00);
    EXPECT_EQ(groupTest.group, 0xFF);

    standard_address_t deviceTest(0, 0, 0xFF, 0, 0);
    EXPECT_EQ(static_cast<raw_address_t>(deviceTest), 0x000F0000);
    EXPECT_EQ(deviceTest.device, 0x0F);

    standard_address_t subsystemTest(0, 0xFF, 0, 0, 0);
    EXPECT_EQ(static_cast<raw_address_t>(subsystemTest), 0x00700000);
    EXPECT_EQ(subsystemTest.subsystem, 0x07);

    standard_address_t systemTest(0xFF, 0, 0, 0, 0);
    EXPECT_EQ(static_cast<raw_address_t>(systemTest), 0x1F800000);
    EXPECT_EQ(systemTest.system, 0x3F);
}

TEST(StandardAddress, CopySemantics)
{
    standard_address_t original(0x01, 0x02, 0x03, 0x04, 0x05);
    standard_address_t copy(original);
    EXPECT_EQ(static_cast<raw_address_t>(copy), static_cast<raw_address_t>(original));
    EXPECT_EQ(copy.system, original.system);
    EXPECT_EQ(copy.subsystem, original.subsystem);
    EXPECT_EQ(copy.device, original.device);
    EXPECT_EQ(copy.group, original.group);
    EXPECT_EQ(copy.parameter, original.parameter);
}

TEST(StandardAddress, ParameterOverride)
{
    standard_address_t original(0x01, 0x02, 0x03, 0x04, 0x05);
    standard_address_t newAddress(original, 0x06, 0x07);
    EXPECT_EQ(static_cast<raw_address_t>(newAddress), 0x00A30607);
    EXPECT_EQ(newAddress.system, 0x01);
    EXPECT_EQ(newAddress.subsystem, 0x02);
    EXPECT_EQ(newAddress.device, 0x03);
    EXPECT_EQ(newAddress.group, 0x06);
    EXPECT_EQ(newAddress.parameter, 0x07);
}

TEST(StandardAddress, AddressBits)
{
    standard_address_t lowBits(0x01, 0x01, 0x01, 0x01, 0x01);
    EXPECT_EQ(static_cast<raw_address_t>(lowBits), 0x00910101);
    standard_address_t highBits(0x10, 0x04, 0x80, 0x01, 0x01);
    EXPECT_EQ(static_cast<raw_address_t>(lowBits), 0x00910101);
}