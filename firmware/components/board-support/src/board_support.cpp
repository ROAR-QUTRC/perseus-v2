#include "board_support.hpp"

#include <Wire.h>

namespace bsp
{
    void initI2C()
    {
        Wire.begin(I2C_SDA, I2C_SCL, 400000);
    }
}
