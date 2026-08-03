#pragma once

#include <array>
#include <cstddef>

namespace OpenLoco
{
    class FormatArgumentsBuffer
    {
        std::array<std::byte, 16> _buffer{};

    public:
        std::byte* data()
        {
            return _buffer.data();
        }

        const std::byte* data() const
        {
            return _buffer.data();
        }

        size_t capacity() const
        {
            return _buffer.size();
        }
    };
}
