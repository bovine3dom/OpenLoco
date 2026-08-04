#pragma once

#include <algorithm>
#include <bit>
#include <cassert>
#include <compare>
#include <cstdint>

namespace OpenLoco
{
#pragma pack(push, 1)
    class ZoomLevel
    {
        int8_t level{};

    public:
        static constexpr int8_t sixteenfold = -4;
        static constexpr int8_t eightfold = -3;
        static constexpr int8_t quadrupled = -2;
        static constexpr int8_t doubled = -1;
        static constexpr int8_t full = 0;
        static constexpr int8_t half = 1;
        static constexpr int8_t quarter = 2;
        static constexpr int8_t eighth = 3;

        static constexpr int8_t min = sixteenfold;
        static constexpr int8_t max = eighth;
        static constexpr uint8_t count = max - min + 1;

        constexpr ZoomLevel() = default;
        constexpr ZoomLevel(int8_t zoom)
            : level(zoom)
        {
        }

        static constexpr ZoomLevel fromEncoded(const uint8_t encoded)
        {
            return ZoomLevel{ std::clamp(std::bit_cast<int8_t>(encoded), min, max) };
        }

        constexpr uint8_t toEncoded() const
        {
            return std::bit_cast<uint8_t>(level);
        }

        explicit constexpr operator int8_t() const
        {
            return level;
        }

        constexpr auto operator<=>(const ZoomLevel& other) const = default;

        constexpr ZoomLevel operator+(int8_t rhs) const
        {
            return ZoomLevel{ static_cast<int8_t>(level + rhs) };
        }

        constexpr ZoomLevel operator-(int8_t rhs) const
        {
            return ZoomLevel{ static_cast<int8_t>(level - rhs) };
        }

        constexpr ZoomLevel& operator++()
        {
            ++level;
            return *this;
        }

        constexpr ZoomLevel operator++(int)
        {
            const auto previous = *this;
            ++level;
            return previous;
        }

        constexpr ZoomLevel& operator--()
        {
            --level;
            return *this;
        }

        constexpr ZoomLevel operator--(int)
        {
            const auto previous = *this;
            --level;
            return previous;
        }

        constexpr uint8_t index() const
        {
            return static_cast<uint8_t>(level - min);
        }

        // Converts a value from screen space into world space.
        template<typename T>
        constexpr T applyTo(const T value) const
        {
            if (level < 0)
            {
                return static_cast<T>(value >> -level);
            }
            return static_cast<T>(static_cast<int64_t>(value) * (int64_t{ 1 } << level));
        }

        // Converts a value from world space into screen space.
        template<typename T>
        constexpr T applyInversedTo(const T value) const
        {
            if (level < 0)
            {
                return static_cast<T>(static_cast<int64_t>(value) * (int64_t{ 1 } << -level));
            }
            return static_cast<T>(value >> level);
        }

        template<typename T>
        constexpr T applyInversedToCeil(const T value) const
        {
            if (level <= 0)
            {
                return applyInversedTo(value);
            }
            const auto divisor = static_cast<T>(1U << level);
            return static_cast<T>(value / divisor + (value % divisor > 0 ? 1 : 0));
        }
    };
#pragma pack(pop)

    static_assert(sizeof(ZoomLevel) == sizeof(int8_t));
}
