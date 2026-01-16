#pragma once

#include <cmath>

namespace axis_math
{

    /**
     * Apply deadband to an axis value and re-normalize.
     *
     * @param axisValue The raw axis value (typically -1.0 to 1.0)
     * @param deadbandPositive The positive deadband threshold
     * @param deadbandNegative The negative deadband threshold (typically negative)
     * @return The processed axis value with deadband applied and re-normalized
     */
    inline double applyDeadband(double axisValue, double deadbandPositive, double deadbandNegative)
    {
        if (!std::isfinite(deadbandPositive))
            return axisValue;

        bool clamped = axisValue < deadbandPositive && axisValue > deadbandNegative;
        if (clamped)
            return 0.0;

        if (axisValue > deadbandPositive)
            axisValue -= deadbandPositive;
        else if (axisValue < deadbandNegative)
            axisValue -= deadbandNegative;

        // re-normalize the axis value
        if (axisValue > 0.0)
            axisValue /= (1.0 - std::abs(deadbandPositive));
        else
            axisValue /= (1.0 - std::abs(deadbandNegative));

        return axisValue;
    }

    /**
     * Apply button overrides to an axis value.
     *
     * @param axisValue The current axis value
     * @param buttonPositive Whether the positive button is pressed
     * @param buttonNegative Whether the negative button is pressed
     * @return The axis value after button overrides (1.0, -1.0, 0.0, or original)
     */
    inline double applyButtonOverride(double axisValue, bool buttonPositive, bool buttonNegative)
    {
        bool bothButtons = buttonPositive && buttonNegative;
        if (bothButtons)
            return 0.0;

        if (buttonPositive)
            return 1.0;
        if (buttonNegative)
            return -1.0;

        return axisValue;
    }

    /**
     * Apply scaling to an axis value.
     *
     * @param axisValue The axis value to scale
     * @param scaling The scaling factor (NaN defaults to 1.0)
     * @return The scaled axis value
     */
    inline double applyScaling(double axisValue, double scaling)
    {
        if (!std::isfinite(scaling))
            scaling = 1.0;
        return axisValue * scaling;
    }

    /**
     * Resolve the negative deadband value.
     * If negative deadband is NaN, default to the negation of positive deadband.
     *
     * @param deadbandPositive The positive deadband threshold
     * @param deadbandNegative The negative deadband threshold (may be NaN)
     * @return The resolved negative deadband value
     */
    inline double resolveNegativeDeadband(double deadbandPositive, double deadbandNegative)
    {
        if (!std::isfinite(deadbandNegative))
            return -deadbandPositive;
        return deadbandNegative;
    }

    /**
     * Determine threshold comparison result for enable condition.
     *
     * @param axisValue The axis value to compare
     * @param threshold The threshold to compare against (NaN defaults to 0.5)
     * @param isLessThan If true, check if axisValue < threshold; otherwise check >
     * @return True if the enable condition is met
     */
    inline bool checkEnableThreshold(double axisValue, double threshold, bool isLessThan)
    {
        if (!std::isfinite(threshold))
            threshold = 0.5;

        if (isLessThan)
            return axisValue < threshold;
        return axisValue > threshold;
    }

}  // namespace axis_math
