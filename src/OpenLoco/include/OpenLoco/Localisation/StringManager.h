#pragma once

#include "StringId.h"
#include <OpenLoco/Engine/Limits.h>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <utility>

#ifdef small
#error "small is defined, likely by windows.h"
#endif

namespace OpenLoco::StringManager
{
    // Language strings, temporary object strings, and loaded object strings.
    constexpr size_t kLegacyNumStringPointers = 0x246E;
    constexpr size_t kNumStringPointers = kLegacyNumStringPointers + (Limits::kMaxVehicleObjects - S5::Limits::kMaxVehicleObjects);
    static_assert(kNumStringPointers <= static_cast<size_t>(std::numeric_limits<StringId>::max()) + 1);

    constexpr uint8_t kUserStringSize = 32;
    constexpr uint16_t kUserStringsStart = 0x8000;
    constexpr uint16_t kUserStringsEnd = kUserStringsStart + Limits::kMaxUserStrings;

    constexpr uint16_t kMaxTownNames = 345;
    constexpr uint16_t kTownNamesStart = 0x9EE7;
    constexpr uint16_t kTownNamesEnd = kTownNamesStart + kMaxTownNames;

    void reset();
    void setString(StringId id, std::string_view value);
    const char* swapString(StringId id, const char* src);
    const char* getString(StringId id);

    StringId userStringAllocate(char* str, bool mustBeUnique);
    const char* getUserString(StringId id);
    void emptyUserString(StringId stringId);
    bool isUserString(StringId id);
}
