// SPDX-License-Identifier: MIT
#pragma once

#include "CargoDist.h"
#include <cstddef>
#include <span>
#include <vector>

namespace OpenLoco::CargoDist
{
    constexpr size_t kMaxSaveDataSize = 16 * 1024 * 1024;

    std::vector<std::byte> encodeState(const State& state);
    State decodeState(std::span<const std::byte> data);
}
