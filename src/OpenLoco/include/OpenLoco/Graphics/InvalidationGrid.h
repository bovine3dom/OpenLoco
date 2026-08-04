#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace OpenLoco::Gfx
{
    class InvalidationGrid
    {
        uint16_t _blockWidth{};
        uint16_t _blockHeight{};
        uint32_t _columnCount{};
        uint32_t _rowCount{};

        std::vector<uint8_t> _blocks;
        uint32_t _screenWidth{};
        uint32_t _screenHeight{};

    public:
        uint32_t getRowCount() const noexcept;

        uint32_t getColumnCount() const noexcept;

        uint32_t getBlockWidth() const noexcept;

        uint32_t getBlockHeight() const noexcept;

        void reset(int32_t width, int32_t height, uint32_t blockWidth, uint32_t blockHeight) noexcept;

        void invalidate(int32_t left, int32_t top, int32_t right, int32_t bottom) noexcept;

        template<typename F>
        void traverseDirtyCells(F&& func)
        {
            const auto columnCount = _columnCount;
            const auto rowCount = _rowCount;
            const auto blockWidth = _blockWidth;
            const auto blockHeight = _blockHeight;
            auto& blocks = _blocks;

            for (uint32_t row = 0; row < rowCount; row++)
            {
                for (uint32_t column = 0; column < columnCount;)
                {
                    const auto blockOffset = row * columnCount + column;
                    if (blocks[blockOffset] == 0)
                    {
                        column++;
                        continue;
                    }

                    uint32_t rectangleWidth = 1;
                    while (column + rectangleWidth < columnCount && blocks[blockOffset + rectangleWidth] != 0)
                    {
                        rectangleWidth++;
                    }

                    uint32_t rectangleHeight = 1;
                    while (row + rectangleHeight < rowCount)
                    {
                        uint32_t dirtyWidth = 0;
                        const auto nextRowOffset = (row + rectangleHeight) * columnCount + column;
                        while (dirtyWidth < rectangleWidth && blocks[nextRowOffset + dirtyWidth] != 0)
                        {
                            dirtyWidth++;
                        }
                        if (dirtyWidth == 0)
                        {
                            break;
                        }

                        rectangleWidth = dirtyWidth;
                        rectangleHeight++;
                    }

                    for (uint32_t dirtyRow = row; dirtyRow < row + rectangleHeight; dirtyRow++)
                    {
                        const auto dirtyOffset = dirtyRow * columnCount + column;
                        std::fill_n(blocks.begin() + dirtyOffset, rectangleWidth, 0);
                    }

                    const auto left = column * blockWidth;
                    const auto top = row * blockHeight;
                    const auto right = (column + rectangleWidth) * blockWidth;
                    const auto bottom = (row + rectangleHeight) * blockHeight;

                    if (left < _screenWidth && top < _screenHeight)
                    {
                        func(left, top, std::min(right, _screenWidth), std::min(bottom, _screenHeight));
                    }

                    column += rectangleWidth;
                }
            }
        }
    };

} // namespace OpenRCT2
