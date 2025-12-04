#include "board_support.hpp"

#if __has_include(<Wire.h>)
#include <Wire.h>

namespace bsp
{
    void initI2C()
    {
        Wire.begin(I2C_SDA, I2C_SCL, 400000);
    }
}

#endif