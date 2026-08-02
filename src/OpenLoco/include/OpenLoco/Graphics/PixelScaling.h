#pragma once

#include "Gfx.h"
#include <cstddef>
#include <cstdint>
#include <span>

namespace OpenLoco::Gfx
{
    // Pitches are measured in palette indices. Source and destination must not overlap.
    [[nodiscard]] bool scaleMmpx2x(
        std::span<const PaletteIndex_t> src,
        int32_t width,
        int32_t height,
        size_t srcPitch,
        std::span<PaletteIndex_t> dst,
        size_t dstPitch,
        std::span<const PaletteEntry> palette);

    // Pitches are measured in their span's element type. Output is opaque ARGB8888.
    // Source and destination must not overlap.
    [[nodiscard]] bool scaleHqx(
        std::span<const PaletteIndex_t> src,
        int32_t width,
        int32_t height,
        size_t srcPitch,
        std::span<uint32_t> dst,
        size_t dstPitch,
        std::span<const PaletteEntry> palette,
        uint8_t factor);
}
