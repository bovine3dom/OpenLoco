#pragma once

#include <cstdint>

namespace OpenLoco
{
    using StringId = uint16_t;

    namespace StringIds
    {
        constexpr StringId empty = 0;
        constexpr StringId dropdown = 96;
        constexpr StringId stepper_plus = 2133;
        constexpr StringId stepper_minus = 2134;
        constexpr StringId null = 0xFFFF;
    }
}
