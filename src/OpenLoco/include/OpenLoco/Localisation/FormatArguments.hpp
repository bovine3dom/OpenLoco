#pragma once

#include "FormatArgumentsBuffer.hpp"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>

namespace OpenLoco
{
    class FormatArguments
    {
    private:
        static inline std::byte _mapTooltipBuffer[40]{};
        static inline uint8_t _mapTooltipTransportMode = 0xFF;

        std::byte* _buffer;
        std::byte* _bufferStart;
        size_t _capacity;
        uint8_t _transportMode = 0xFF;

    public:
        FormatArguments(std::byte* buffer, size_t length)
        {
            _bufferStart = buffer;
            _buffer = _bufferStart;
            _capacity = length;
        }

        FormatArguments(FormatArgumentsBuffer& buffer)
        {
            _buffer = buffer.data();
            _bufferStart = _buffer;
            _capacity = buffer.capacity();
        }

        FormatArguments(const FormatArgumentsBuffer& buffer)
        {
            // FIXME: Create a view type for FormatArgumentsBuffer.
            _buffer = const_cast<std::byte*>(buffer.data());
            _bufferStart = _buffer;
            _capacity = buffer.capacity();
        }

        FormatArguments()
        {
            // TODO: refactor users to use non-static buffers
            static std::byte defaultBuffer[20];
            _bufferStart = _buffer = &*defaultBuffer;
            _capacity = std::size(defaultBuffer);
        }

        template<typename... T>
        static FormatArguments common(T&&... args)
        {
            // TODO: refactor users to use non-static buffers
            FormatArguments formatter;
            (formatter.push(args), ...);
            return formatter;
        }

        static FormatArguments mapToolTip()
        {
            // TODO: refactor users to use non-static buffers
            FormatArguments formatter{ _mapTooltipBuffer, std::size(_mapTooltipBuffer) };
            formatter._transportMode = _mapTooltipTransportMode;
            return formatter;
        }

        template<typename... T>
        static FormatArguments mapToolTip(T&&... args)
        {
            // TODO: refactor users to use non-static buffers
            auto formatter = FormatArguments::mapToolTip();
            formatter.setTransportMode(0xFF);
            (formatter.push(args), ...);
            return formatter;
        }

        // Size in bytes to skip forward the buffer
        void skip(const size_t size)
        {
            auto* const nextOffset = getNextOffset(size);

            _buffer = nextOffset;
        }

        template<typename T>
        void push(T arg)
        {
            static_assert(sizeof(T) % 2 == 0, "Tried to push an odd number of bytes onto the format args!");
            auto* nextOffset = getNextOffset(sizeof(T));

            *(T*)_buffer = arg;
            _buffer = nextOffset;
        }

        void rewind()
        {
            _buffer = _bufferStart;
        }

        void setTransportMode(uint8_t mode)
        {
            _transportMode = mode;
            if (_bufferStart == _mapTooltipBuffer)
            {
                _mapTooltipTransportMode = mode;
            }
        }
        uint8_t getTransportMode() const { return _transportMode; }

        const void* operator&() const
        {
            return _bufferStart;
        }

        const std::byte* getBufferStart() const
        {
            return _bufferStart;
        }

        size_t getLength() const
        {
            return _buffer - _bufferStart;
        }

        size_t getCapacity() const
        {
            return _capacity;
        }

    private:
        std::byte* getNextOffset(size_t size) const;
    };

    class FormatArgumentsView
    {
    private:
        const std::byte* args{};
        const std::byte* end{};
        uint8_t transportMode = 0xFF;

    public:
        constexpr FormatArgumentsView() = default;

        FormatArgumentsView(const FormatArguments& newargs)
            : args(newargs.getBufferStart())
            , end(newargs.getBufferStart() + newargs.getCapacity())
            , transportMode(newargs.getTransportMode()) {};

        FormatArgumentsView(const FormatArgumentsBuffer& newargs)
            : args(newargs.data())
            , end(newargs.data() + newargs.capacity()) {};

        template<typename T>
        T pop()
        {
            if (args == nullptr || end == nullptr || args > end || static_cast<size_t>(end - args) < sizeof(T))
            {
                args = end;
                return T{};
            }

            T value;
            std::memcpy(&value, args, sizeof(T));
            args += sizeof(T);

            return value;
        }

        template<typename T>
        void skip()
        {
            if (args == nullptr || end == nullptr || args > end || static_cast<size_t>(end - args) < sizeof(T))
            {
                args = end;
                return;
            }
            args += sizeof(T);
        }

        template<typename T>
        void push()
        {
            args -= sizeof(T);
        }

        uint8_t getTransportMode() const { return transportMode; }
    };
}
