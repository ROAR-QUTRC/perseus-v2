#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "generic_controller/axis_math.hpp"

using namespace axis_math;

class DeadbandTest : public ::testing::Test
{
protected:
    static constexpr double DEFAULT_DEADBAND = 0.08;
};

// Deadband Tests

TEST_F(DeadbandTest, ValueWithinDeadbandReturnsZero)
{
    EXPECT_DOUBLE_EQ(0.0, applyDeadband(0.05, DEFAULT_DEADBAND, -DEFAULT_DEADBAND));
    EXPECT_DOUBLE_EQ(0.0, applyDeadband(-0.05, DEFAULT_DEADBAND, -DEFAULT_DEADBAND));
    EXPECT_DOUBLE_EQ(0.0, applyDeadband(0.0, DEFAULT_DEADBAND, -DEFAULT_DEADBAND));
}

TEST_F(DeadbandTest, ValueAtDeadbandBoundaryReturnsZero)
{
    // Values exactly at deadband boundary should still be in deadband (< not <=)
    double result = applyDeadband(DEFAULT_DEADBAND - 0.001, DEFAULT_DEADBAND, -DEFAULT_DEADBAND);
    EXPECT_DOUBLE_EQ(0.0, result);
}

TEST_F(DeadbandTest, ValueAboveDeadbandIsNormalized)
{
    // Value of 1.0 with 0.08 deadband should normalize to 1.0
    double result = applyDeadband(1.0, DEFAULT_DEADBAND, -DEFAULT_DEADBAND);
    EXPECT_NEAR(1.0, result, 0.001);

    // Value of 0.5 with 0.08 deadband should normalize to (0.5 - 0.08) / (1 - 0.08)
    result = applyDeadband(0.5, DEFAULT_DEADBAND, -DEFAULT_DEADBAND);
    double expected = (0.5 - DEFAULT_DEADBAND) / (1.0 - DEFAULT_DEADBAND);
    EXPECT_NEAR(expected, result, 0.001);
}

TEST_F(DeadbandTest, ValueBelowNegativeDeadbandIsNormalized)
{
    // Value of -1.0 with -0.08 deadband should normalize to -1.0
    double result = applyDeadband(-1.0, DEFAULT_DEADBAND, -DEFAULT_DEADBAND);
    EXPECT_NEAR(-1.0, result, 0.001);

    // Value of -0.5 with -0.08 deadband should normalize to (-0.5 - (-0.08)) / (1 - 0.08)
    result = applyDeadband(-0.5, DEFAULT_DEADBAND, -DEFAULT_DEADBAND);
    double expected = (-0.5 - (-DEFAULT_DEADBAND)) / (1.0 - DEFAULT_DEADBAND);
    EXPECT_NEAR(expected, result, 0.001);
}

TEST_F(DeadbandTest, AsymmetricDeadband)
{
    // Different positive and negative deadbands
    double posDeadband = 0.1;
    double negDeadband = -0.2;

    // Within positive but outside negative side
    EXPECT_DOUBLE_EQ(0.0, applyDeadband(0.05, posDeadband, negDeadband));

    // Within negative but outside positive side
    EXPECT_DOUBLE_EQ(0.0, applyDeadband(-0.15, posDeadband, negDeadband));

    // Above positive deadband
    double result = applyDeadband(0.5, posDeadband, negDeadband);
    double expected = (0.5 - posDeadband) / (1.0 - posDeadband);
    EXPECT_NEAR(expected, result, 0.001);

    // Below negative deadband
    result = applyDeadband(-0.5, posDeadband, negDeadband);
    expected = (-0.5 - negDeadband) / (1.0 - std::abs(negDeadband));
    EXPECT_NEAR(expected, result, 0.001);
}

TEST_F(DeadbandTest, NaNDeadbandReturnsOriginalValue)
{
    double nanDeadband = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(0.5, applyDeadband(0.5, nanDeadband, -0.08));
    EXPECT_DOUBLE_EQ(-0.3, applyDeadband(-0.3, nanDeadband, -0.08));
}

TEST_F(DeadbandTest, ZeroDeadbandPassesThrough)
{
    EXPECT_DOUBLE_EQ(0.5, applyDeadband(0.5, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(-0.5, applyDeadband(-0.5, 0.0, 0.0));
    // Zero should still be clamped since 0 < 0 is false and 0 > 0 is false
    // Actually with deadband=0, the condition is value < 0 && value > 0 which is always false
    // so no clamping happens
    EXPECT_DOUBLE_EQ(0.0, applyDeadband(0.0, 0.0, 0.0));
}

// Button Override Tests

class ButtonOverrideTest : public ::testing::Test
{
};

TEST_F(ButtonOverrideTest, NoButtonsReturnsOriginalValue)
{
    EXPECT_DOUBLE_EQ(0.5, applyButtonOverride(0.5, false, false));
    EXPECT_DOUBLE_EQ(-0.3, applyButtonOverride(-0.3, false, false));
    EXPECT_DOUBLE_EQ(0.0, applyButtonOverride(0.0, false, false));
}

TEST_F(ButtonOverrideTest, PositiveButtonReturnsOne)
{
    EXPECT_DOUBLE_EQ(1.0, applyButtonOverride(0.0, true, false));
    EXPECT_DOUBLE_EQ(1.0, applyButtonOverride(0.5, true, false));
    EXPECT_DOUBLE_EQ(1.0, applyButtonOverride(-0.5, true, false));
}

TEST_F(ButtonOverrideTest, NegativeButtonReturnsNegativeOne)
{
    EXPECT_DOUBLE_EQ(-1.0, applyButtonOverride(0.0, false, true));
    EXPECT_DOUBLE_EQ(-1.0, applyButtonOverride(0.5, false, true));
    EXPECT_DOUBLE_EQ(-1.0, applyButtonOverride(-0.5, false, true));
}

TEST_F(ButtonOverrideTest, BothButtonsReturnsZero)
{
    EXPECT_DOUBLE_EQ(0.0, applyButtonOverride(0.0, true, true));
    EXPECT_DOUBLE_EQ(0.0, applyButtonOverride(0.5, true, true));
    EXPECT_DOUBLE_EQ(0.0, applyButtonOverride(-0.5, true, true));
}

// Scaling Tests

class ScalingTest : public ::testing::Test
{
};

TEST_F(ScalingTest, BasicScaling)
{
    EXPECT_DOUBLE_EQ(1.4, applyScaling(0.7, 2.0));
    EXPECT_DOUBLE_EQ(-0.5, applyScaling(1.0, -0.5));
    EXPECT_DOUBLE_EQ(0.0, applyScaling(0.0, 5.0));
}

TEST_F(ScalingTest, NaNScalingDefaultsToOne)
{
    double nanScaling = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(0.5, applyScaling(0.5, nanScaling));
    EXPECT_DOUBLE_EQ(-0.3, applyScaling(-0.3, nanScaling));
}

TEST_F(ScalingTest, ZeroScalingReturnsZero)
{
    EXPECT_DOUBLE_EQ(0.0, applyScaling(0.5, 0.0));
    EXPECT_DOUBLE_EQ(0.0, applyScaling(-1.0, 0.0));
}

TEST_F(ScalingTest, NegativeScalingInvertsAxis)
{
    EXPECT_DOUBLE_EQ(-0.5, applyScaling(0.5, -1.0));
    EXPECT_DOUBLE_EQ(0.5, applyScaling(-0.5, -1.0));
}

// Resolve Negative Deadband Tests

class ResolveNegativeDeadbandTest : public ::testing::Test
{
};

TEST_F(ResolveNegativeDeadbandTest, FiniteNegativeDeadbandReturnsAsIs)
{
    EXPECT_DOUBLE_EQ(-0.1, resolveNegativeDeadband(0.08, -0.1));
    EXPECT_DOUBLE_EQ(-0.2, resolveNegativeDeadband(0.08, -0.2));
}

TEST_F(ResolveNegativeDeadbandTest, NaNNegativeDeadbandDefaultsToNegativePositive)
{
    double nanDeadband = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(-0.08, resolveNegativeDeadband(0.08, nanDeadband));
    EXPECT_DOUBLE_EQ(-0.15, resolveNegativeDeadband(0.15, nanDeadband));
}

// Enable Threshold Tests

class EnableThresholdTest : public ::testing::Test
{
};

TEST_F(EnableThresholdTest, GreaterThanThreshold)
{
    // isLessThan = false means check if value > threshold
    EXPECT_TRUE(checkEnableThreshold(0.6, 0.5, false));
    EXPECT_FALSE(checkEnableThreshold(0.4, 0.5, false));
    EXPECT_FALSE(checkEnableThreshold(0.5, 0.5, false));  // exactly at threshold
}

TEST_F(EnableThresholdTest, LessThanThreshold)
{
    // isLessThan = true means check if value < threshold
    EXPECT_TRUE(checkEnableThreshold(0.4, 0.5, true));
    EXPECT_FALSE(checkEnableThreshold(0.6, 0.5, true));
    EXPECT_FALSE(checkEnableThreshold(0.5, 0.5, true));  // exactly at threshold
}

TEST_F(EnableThresholdTest, NaNThresholdDefaultsToHalf)
{
    double nanThreshold = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(checkEnableThreshold(0.6, nanThreshold, false));   // 0.6 > 0.5
    EXPECT_FALSE(checkEnableThreshold(0.4, nanThreshold, false));  // 0.4 > 0.5
    EXPECT_TRUE(checkEnableThreshold(0.4, nanThreshold, true));    // 0.4 < 0.5
    EXPECT_FALSE(checkEnableThreshold(0.6, nanThreshold, true));   // 0.6 < 0.5
}

TEST_F(EnableThresholdTest, NegativeThreshold)
{
    // Trigger axis typically uses negative threshold (e.g., -0.5 for pulled trigger)
    EXPECT_TRUE(checkEnableThreshold(-0.6, -0.5, true));   // -0.6 < -0.5
    EXPECT_FALSE(checkEnableThreshold(-0.4, -0.5, true));  // -0.4 < -0.5
}

// Integration Tests - Full Pipeline

class FullPipelineTest : public ::testing::Test
{
protected:
    double processAxis(double rawValue, double deadbandPos, double deadbandNeg,
                       bool buttonPos, bool buttonNeg, double scaling)
    {
        double value = rawValue;
        double negDeadband = resolveNegativeDeadband(deadbandPos, deadbandNeg);
        value = applyDeadband(value, deadbandPos, negDeadband);
        value = applyButtonOverride(value, buttonPos, buttonNeg);
        value = applyScaling(value, scaling);
        return value;
    }
};

TEST_F(FullPipelineTest, TypicalJoystickInput)
{
    // Simulate typical joystick input with 0.08 deadband and 0.7 scaling
    double raw = 0.5;
    double result = processAxis(raw, 0.08, -0.08, false, false, 0.7);

    // Expected: (0.5 - 0.08) / (1 - 0.08) * 0.7
    double expectedNormalized = (0.5 - 0.08) / (1.0 - 0.08);
    double expected = expectedNormalized * 0.7;
    EXPECT_NEAR(expected, result, 0.001);
}

TEST_F(FullPipelineTest, ButtonOverridesDeadband)
{
    // Even with value in deadband, button press should override
    double raw = 0.01;  // Within deadband
    double result = processAxis(raw, 0.08, -0.08, true, false, 0.7);
    EXPECT_DOUBLE_EQ(0.7, result);  // 1.0 * 0.7
}

TEST_F(FullPipelineTest, SmallInputInDeadbandIsZero)
{
    double raw = 0.05;  // Small input within deadband
    double result = processAxis(raw, 0.08, -0.08, false, false, 2.0);
    EXPECT_DOUBLE_EQ(0.0, result);
}
