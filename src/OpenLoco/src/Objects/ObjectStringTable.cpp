#include "Objects/ObjectStringTable.h"
#include "Config.h"
#include "Localisation/Languages.h"
#include "Localisation/StringIds.h"
#include "Localisation/StringManager.h"
#include "Objects/ObjectManager.h"
#include <OpenLoco/Core/Exception.hpp>

namespace OpenLoco::ObjectManager
{
    constexpr std::array<StringId, 16> kTemporaryObjectStringIds = {
        StringIds::temporary_object_load_str_0,
        StringIds::temporary_object_load_str_1,
        StringIds::temporary_object_load_str_2,
        StringIds::temporary_object_load_str_3,
        StringIds::temporary_object_load_str_4,
        StringIds::temporary_object_load_str_5,
        StringIds::temporary_object_load_str_6,
        StringIds::temporary_object_load_str_7,
        StringIds::temporary_object_load_str_8,
        StringIds::temporary_object_load_str_9,
        StringIds::temporary_object_load_str_10,
        StringIds::temporary_object_load_str_11,
        StringIds::temporary_object_load_str_12,
        StringIds::temporary_object_load_str_13,
        StringIds::temporary_object_load_str_14,
        StringIds::temporary_object_load_str_15,
    };

    constexpr std::array<uint8_t, kMaxObjectTypes> kNumStringsPerObjectType = {
        1, // interface,
        1, // sound,
        3, // currency,
        1, // steam,
        1, // rock,
        1, // water,
        1, // surface,
        1, // townNames,
        4, // cargo,
        1, // wall,
        2, // train_signal,
        1, // levelCrossing,
        1, // streetLight,
        1, // tunnel,
        1, // bridge,
        1, // train_station,
        1, // trackExtra,
        1, // track,
        1, // roadStation,
        1, // roadExtra,
        1, // road,
        1, // airport,
        1, // dock,
        1, // vehicle,
        1, // tree,
        1, // snow,
        1, // climate,
        1, // hillShapes,
        1, // building,
        1, // scaffolding,
        8, // industry,
        1, // region,
        2, // competitor,
        2, // scenarioText,
    };

    constexpr size_t getLegacyMaxObjects(const ObjectType type)
    {
        return type == ObjectType::vehicle ? S5::Limits::kMaxVehicleObjects : getMaxObjects(type);
    }

    constexpr size_t getNumLegacyObjectStrings()
    {
        size_t count = 0;
        for (uint8_t type = 0; type < kMaxObjectTypes; ++type)
        {
            count += getLegacyMaxObjects(static_cast<ObjectType>(type)) * kNumStringsPerObjectType[type];
        }
        return count;
    }
    static_assert(StringIds::object_strings_begin + getNumLegacyObjectStrings() == StringManager::kLegacyNumStringPointers);

    // 0x00472172
    StringTableResult loadStringTable(std::span<const std::byte> data, const LoadedObjectHandle& handle, uint8_t index)
    {
        const auto objectType = enumValue(handle.type);
        if (objectType >= kNumStringsPerObjectType.size()
            || index >= kNumStringsPerObjectType[objectType]
            || handle.id >= getMaxObjects(handle.type))
        {
            throw Exception::OutOfRange();
        }

        StringTableResult res;
        auto iter = data.begin();
        const char* engBackupStr = nullptr;
        const char* anyStr = nullptr;
        const char* targetStr = nullptr;
        const auto targetLang = Localisation::getDescriptorForLanguage(Config::get().language).locoOriginalId;
        for (; iter != data.end() && *iter != static_cast<std::byte>(0xFF); ++iter)
        {
            const auto lang = static_cast<Localisation::LocoLanguageId>(*iter++);
            const auto str = reinterpret_cast<const char*>(&*iter);
            if (lang == Localisation::LocoLanguageId::english_uk)
            {
                engBackupStr = str;
            }
            else if (lang == Localisation::LocoLanguageId::english_us && engBackupStr == nullptr)
            {
                engBackupStr = str;
            }
            if (lang == targetLang)
            {
                targetStr = str;
            }
            if (engBackupStr == nullptr && targetStr == nullptr)
            {
                anyStr = str;
            }
            iter += strlen(str);
        }
        iter++;
        res.tableLength = std::distance(data.begin(), iter);
        const auto* chosenStr = [=]() {
            if (targetStr != nullptr)
            {
                return targetStr;
            }
            if (engBackupStr != nullptr)
            {
                return engBackupStr;
            }
            return anyStr;
        }();

        if (isTemporaryObjectLoad())
        {
            res.str = kTemporaryObjectStringIds[index];
            StringManager::swapString(res.str, chosenStr);
            return res;
        }

        size_t stringId;
        if (handle.type == ObjectType::vehicle && handle.id >= S5::Limits::kMaxVehicleObjects)
        {
            stringId = StringManager::kLegacyNumStringPointers + (handle.id - S5::Limits::kMaxVehicleObjects) * kNumStringsPerObjectType[objectType] + index;
        }
        else
        {
            stringId = StringIds::object_strings_begin + index;
            for (auto objType = ObjectType::interfaceSkin; enumValue(objType) < enumValue(handle.type); objType = static_cast<ObjectType>(enumValue(objType) + 1))
            {
                stringId += getLegacyMaxObjects(objType) * kNumStringsPerObjectType[enumValue(objType)];
            }
            stringId += kNumStringsPerObjectType[objectType] * handle.id;
        }
        if (stringId >= StringManager::kNumStringPointers)
        {
            throw Exception::OutOfRange();
        }
        res.str = static_cast<StringId>(stringId);

        StringManager::swapString(res.str, chosenStr);
        return res;
    }
}
