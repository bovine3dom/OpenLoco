#include "Objects/VehicleObject.h"
#include "Graphics/Colour.h"
#include "Graphics/DrawingContext.h"
#include "Graphics/Gfx.h"
#include "Graphics/TextRenderer.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/Formatting.h"
#include "Localisation/StringIds.h"
#include "Logging.h"
#include "Objects/CargoObject.h"
#include "Objects/ObjectImageTable.h"
#include "Objects/ObjectManager.h"
#include "Objects/ObjectStringTable.h"
#include "Ui/WindowManager.h"
#include "Vehicles/VehicleDraw.h"
#include <OpenLoco/Core/Exception.hpp>
#include <OpenLoco/Core/Numerics.hpp>
#include <algorithm>
#include <cstring>

using namespace OpenLoco::Diagnostics;

namespace OpenLoco
{
    struct VehicleCapacityOverride
    {
        std::string_view name;
        uint8_t originalCapacity;
        uint8_t correctedCapacity;
    };

    // Normalize well-identified vehicles to published seating or normal service capacity
    // without modifying their DAT data, checksum, or save identity.
    static constexpr VehicleCapacityOverride kCapacityOverrides[] = {
        { "142     ", 90, 121 },
        { "2EPB    ", 100, 186 },
        { "AILSA1  ", 60, 79 },
        { "CLASSIC ", 45, 78 },
        { "COMET   ", 90, 44 },
        { "CONCOR  ", 250, 100 },
        { "ESTAR2  ", 67, 44 },
        { "HCRAFT1 ", 200, 254 },
        { "JFOIL1  ", 125, 250 },
        { "LEOP1   ", 45, 75 },
        { "RBE24   ", 50, 100 },
        { "RTMASTER", 50, 69 },
        { "TDH5301 ", 40, 75 },
        { "TGV2    ", 70, 48 },
        { "TRAM1   ", 35, 102 },
        { "TRAM2   ", 20, 33 },
        { "TRAM3   ", 55, 74 },
        { "TRAM4   ", 65, 120 },
        { "TRAMCOMB", 85, 176 },
        { "VULCAN  ", 14, 28 },
    };

    static const VehicleCapacityOverride* findVehicleCapacityOverride(const ObjectHeader& header)
    {
        if (header.getType() != ObjectType::vehicle || header.getSourceGame() != SourceGame::vanilla)
        {
            return nullptr;
        }

        for (const auto& capacityOverride : kCapacityOverrides)
        {
            if (header.getName() == capacityOverride.name)
            {
                return &capacityOverride;
            }
        }
        return nullptr;
    }

    bool isOfficialTgvPassengerCarriage(const ObjectHeader& header)
    {
        return header.flags == kOfficialTgvPassengerCarriageHeader.flags
            && header.getName() == kOfficialTgvPassengerCarriageHeader.getName()
            && header.checksum == kOfficialTgvPassengerCarriageHeader.checksum;
    }

    bool isOfficialMailCargo(const ObjectHeader& header)
    {
        return header.flags == kOfficialMailCargoHeader.flags
            && header.getName() == kOfficialMailCargoHeader.getName()
            && header.checksum == kOfficialMailCargoHeader.checksum;
    }

    bool isTgvLaPosteObject(const ObjectHeader& header)
    {
        return header.flags == kTgvLaPosteObjectHeader.flags
            && header.getName() == kTgvLaPosteObjectHeader.getName()
            && header.checksum == kTgvLaPosteObjectHeader.checksum;
    }

    bool isTgvLaPosteObject(const LoadedObjectId objectId)
    {
        return objectId < ObjectManager::getMaxObjects(ObjectType::vehicle)
            && ObjectManager::get<VehicleObject>(objectId) != nullptr
            && isTgvLaPosteObject(ObjectManager::getHeader({ ObjectType::vehicle, objectId }));
    }

    void applyTgvLaPosteVehicleOverrides(VehicleObject& vehicle, const uint32_t mailCargoMask)
    {
        vehicle.name = StringIds::tgv_la_poste_mail_carriage;
        vehicle.maxCargo[0] = mailCargoMask == 0 ? 0 : kTgvLaPosteMailCapacity;
        vehicle.maxCargo[1] = 0;
        vehicle.compatibleCargoCategories[0] = mailCargoMask;
        vehicle.compatibleCargoCategories[1] = 0;
        vehicle.numSimultaneousCargoTypes = mailCargoMask == 0 ? 0 : 1;
        std::fill(std::begin(vehicle.cargoTypeSpriteOffsets), std::end(vehicle.cargoTypeSpriteOffsets), 0);
        vehicle.flags &= ~VehicleObjectFlags::refittable;
        vehicle.flags |= VehicleObjectFlags::quietInvention;
    }

    ColourScheme getEffectiveVehicleColourScheme(const ObjectHeader& header, const ColourScheme requested)
    {
        return isTgvLaPosteObject(header)
            ? ColourScheme{ Colour::yellow, Colour::yellow }
            : requested;
    }

    ColourScheme getEffectiveVehicleColourScheme(const LoadedObjectId objectId, const ColourScheme requested)
    {
        return isTgvLaPosteObject(objectId)
            ? getEffectiveVehicleColourScheme(ObjectManager::getHeader({ ObjectType::vehicle, objectId }), requested)
            : requested;
    }

    bool isSrn4HovercraftObject(const ObjectHeader& header)
    {
        return header.getType() == ObjectType::vehicle
            && header.getSourceGame() == SourceGame::vanilla
            && header.getName() == "HCRAFT1 ";
    }

    bool isSrn4HovercraftObject(const LoadedObjectId objectId)
    {
        if (objectId >= ObjectManager::getMaxObjects(ObjectType::vehicle)
            || ObjectManager::get<VehicleObject>(objectId) == nullptr)
        {
            return false;
        }
        return isSrn4HovercraftObject(ObjectManager::getHeader({ ObjectType::vehicle, objectId }));
    }

    // 0x004B8C52
    void VehicleObject::drawPreviewImage(Gfx::DrawingContext& drawingCtx, const int16_t x, const int16_t y) const
    {
        uint8_t yaw = Ui::WindowManager::getVehiclePreviewRotationFrameYaw();
        uint8_t roll = Ui::WindowManager::getVehiclePreviewRotationFrameRoll();

        const auto colour = name == StringIds::tgv_la_poste_mail_carriage
            ? ColourScheme{ Colour::yellow, Colour::yellow }
            : ColourScheme{ Colour::mutedSeaGreen, Colour::white };
        drawVehicleOverview(drawingCtx, Ui::Point{ x, y } + Ui::Point{ 0, 19 }, *this, yaw, roll, colour);
    }

    // TODO: Should only be defined in ObjectSelectionWindow
    static constexpr uint8_t kDescriptionRowHeight = 10;

    // 0x004B8C9D
    void VehicleObject::drawDescription(Gfx::DrawingContext& drawingCtx, const int16_t x, const int16_t y, const int16_t width) const
    {
        auto tr = Gfx::TextRenderer(drawingCtx);

        Ui::Point rowPosition = { x, y };
        ObjectManager::drawGenericDescription(drawingCtx, rowPosition, designed, obsolete);
        if (power != 0 && (mode == TransportMode::road || mode == TransportMode::rail))
        {
            FormatArguments args{};
            args.push(power);
            tr.drawStringLeft(rowPosition, Colour::black, StringIds::object_selection_power, args);
            rowPosition.y += kDescriptionRowHeight;
        }
        {
            FormatArguments args{};
            args.push<uint32_t>(StringManager::internalLengthToComma1DP(getLength()));
            tr.drawStringLeft(rowPosition, Colour::black, StringIds::object_selection_length, args);
            rowPosition.y += kDescriptionRowHeight;
        }
        {
            FormatArguments args{};
            args.push(weight);
            tr.drawStringLeft(rowPosition, Colour::black, StringIds::object_selection_weight, args);
            rowPosition.y += kDescriptionRowHeight;
        }
        {
            FormatArguments args{};
            args.setTransportMode(enumValue(mode));
            args.push(speed);
            tr.drawStringLeft(rowPosition, Colour::black, StringIds::object_selection_max_speed, args);
        }
        auto buffer = const_cast<char*>(StringManager::getString(StringIds::buffer_1250));
        // Clear buffer
        *buffer = '\0';

        getCargoString(buffer, StringManager::kTemporaryStringSize);

        if (StringManager::locoStrlen(buffer) != 0)
        {
            tr.drawStringLeftWrapped(rowPosition, width - 4, Colour::black, StringIds::buffer_1250);
        }
    }

    char* VehicleObject::getCargoString(char* buffer, const size_t bufferLen) const
    {
        auto* const bufferStart = buffer;
        const auto getRemaining = [&]() {
            return bufferLen - std::min<size_t>(buffer - bufferStart, bufferLen);
        };
        const auto appendString = [&](const StringId stringId, const FormatArgumentsView args = {}) {
            const auto remaining = getRemaining();
            if (remaining <= 1)
            {
                return;
            }
            char formatted[StringManager::kTemporaryStringSize]{};
            try
            {
                StringManager::formatString(formatted, std::size(formatted), stringId, args);
            }
            catch (const Exception::OverflowError&)
            {
                return;
            }
            const auto length = StringManager::locoStrlenS(formatted, remaining - 1);
            std::memcpy(buffer, formatted, length);
            buffer += length;
            *buffer = '\0';
        };
        const auto appendText = [&](const std::string_view text) {
            const auto remaining = getRemaining();
            if (remaining == 0)
            {
                return;
            }
            const auto length = std::min(text.size(), remaining - 1);
            std::memcpy(buffer, text.data(), length);
            buffer += length;
            *buffer = '\0';
        };
        const auto appendCompartment = [&](const uint8_t compartment, const StringId capacityString) {
            auto cargoTypes = compatibleCargoCategories[compartment];
            const auto defaultCargoType = Numerics::bitScanForward(cargoTypes);
            if (defaultCargoType == -1)
            {
                return;
            }
            cargoTypes &= ~(1U << defaultCargoType);

            const auto* cargoObj = ObjectManager::get<CargoObject>(defaultCargoType);
            FormatArguments capacityArgs{};
            capacityArgs.push(maxCargo[compartment] == 1 ? cargoObj->unitNameSingular : cargoObj->unitNamePlural);
            capacityArgs.push<uint32_t>(maxCargo[compartment]);
            appendString(capacityString, capacityArgs);

            if (cargoTypes == 0)
            {
                return;
            }
            appendText(" (");
            bool isFirst = true;
            while (cargoTypes != 0)
            {
                const auto cargoType = Numerics::bitScanForward(cargoTypes);
                cargoTypes &= ~(1U << cargoType);
                if (!isFirst)
                {
                    appendText(" ");
                }
                const auto* alternativeCargoObj = ObjectManager::get<CargoObject>(cargoType);
                FormatArguments cargoArgs{};
                cargoArgs.push(alternativeCargoObj->name);
                appendString(StringIds::stats_or_string, cargoArgs);
                isFirst = false;
            }
            appendText(")");
        };

        if (numSimultaneousCargoTypes != 0)
        {
            appendCompartment(0, StringIds::stats_capacity);

            if (hasFlags(VehicleObjectFlags::refittable))
            {
                appendString(StringIds::stats_refittable);
            }

            if (numSimultaneousCargoTypes > 1)
            {
                appendCompartment(1, StringIds::stats_plus_string);
            }
        }
        return buffer;
    }

    // 0x004B8B23
    bool VehicleObject::validate() const
    {
        if (costIndex > 32)
        {
            return false;
        }
        if (runCostIndex > 32)
        {
            return false;
        }

        if (costFactor <= 0)
        {
            return false;
        }
        if (runCostFactor < 0)
        {
            return false;
        }

        if (hasFlags(VehicleObjectFlags::anyRoadType))
        {
            if (numTrackExtras != 0)
            {
                return false;
            }
            if (hasFlags(VehicleObjectFlags::rackRail))
            {
                return false;
            }
        }

        if (numTrackExtras > 4)
        {
            return false;
        }

        if (numSimultaneousCargoTypes > 2)
        {
            return false;
        }

        if (numCompatibleVehicles > 8)
        {
            return false;
        }

        if (rackSpeed > speed)
        {
            return false;
        }

        for (const auto& bodySprite : bodySprites)
        {
            if (!bodySprite.hasFlags(BodySpriteFlags::hasSprites))
            {
                continue;
            }

            switch (bodySprite.numFlatRotationFrames)
            {
                case 8:
                case 16:
                case 32:
                case 64:
                case 128:
                    break;
                default:
                    return false;
            }
            switch (bodySprite.numSlopedRotationFrames)
            {
                case 4:
                case 8:
                case 16:
                case 32:
                    break;
                default:
                    return false;
            }
            switch (bodySprite.numAnimationFrames)
            {
                case 1:
                case 2:
                case 4:
                    break;
                default:
                    return false;
            }
            if (bodySprite.numCargoLoadFrames < 1 || bodySprite.numCargoLoadFrames > 5)
            {
                return false;
            }
            switch (bodySprite.numRollFrames)
            {
                case 1:
                case 3:
                    break;
                default:
                    return false;
            }
        }

        for (auto& bogieSprite : bogieSprites)
        {
            if (!bogieSprite.hasFlags(BogieSpriteFlags::hasSprites))
            {
                continue;
            }

            switch (bogieSprite.numAnimationFrames)
            {
                case 1:
                case 2:
                case 4:
                    break;
                default:
                    return false;
            }
        }

        const auto startSoundCount = numStartSounds & NumStartSounds::kMask;
        if (startSoundCount > kMaxStartSounds)
        {
            return false;
        }

        return true;
    }

    static constexpr uint8_t getYawAccuracyFlat(uint8_t numFrames)
    {
        switch (numFrames)
        {
            case 8:
                return 1;
            case 16:
                return 2;
            case 32:
                return 3;
            default:
                return 4;
        }
    }

    static constexpr uint8_t getYawAccuracySloped(uint8_t numFrames)
    {
        switch (numFrames)
        {
            case 4:
                return 0;
            case 8:
                return 1;
            case 16:
                return 2;
            default:
                return 3;
        }
    }

    // 0x004B841B
    void VehicleObject::load(const LoadedObjectHandle& handle, [[maybe_unused]] std::span<const std::byte> data, ObjectManager::DependentObjects* dependencies)
    {
        auto remainingData = data.subspan(sizeof(VehicleObject));

        auto strRes = ObjectManager::loadStringTable(remainingData, handle, 0);
        name = strRes.str;
        remainingData = remainingData.subspan(strRes.tableLength);

        trackType = 0xFF;
        if (!hasFlags(VehicleObjectFlags::anyRoadType) && (mode == TransportMode::rail || mode == TransportMode::road))
        {
            ObjectHeader trackHeader = *reinterpret_cast<const ObjectHeader*>(remainingData.data());
            if (dependencies != nullptr)
            {
                dependencies->required.push_back(trackHeader);
            }
            auto res = ObjectManager::findObjectHandle(trackHeader);
            if (res.has_value())
            {
                trackType = res->id;
            }
            remainingData = remainingData.subspan(sizeof(ObjectHeader));
        }

        // Load Extra
        for (auto i = 0U, index = 0U; i < numTrackExtras; ++i)
        {
            ObjectHeader modHeader = *reinterpret_cast<const ObjectHeader*>(remainingData.data());
            if (dependencies != nullptr)
            {
                dependencies->required.push_back(modHeader);
            }
            auto res = ObjectManager::findObjectHandle(modHeader);
            if (res.has_value())
            {
                requiredTrackExtras[index++] = res->id;
            }
            remainingData = remainingData.subspan(sizeof(ObjectHeader));
        }

        std::fill(std::begin(cargoTypeSpriteOffsets), std::end(cargoTypeSpriteOffsets), 0);
        std::fill(std::begin(compatibleCargoCategories), std::end(compatibleCargoCategories), 0);
        numSimultaneousCargoTypes = 0;

        for (auto i = 0U; i < std::size(compatibleCargoCategories); ++i)
        {
            const auto index = numSimultaneousCargoTypes;
            maxCargo[index] = *reinterpret_cast<const uint8_t*>(remainingData.data());
            remainingData = remainingData.subspan(sizeof(uint8_t));
            if (maxCargo[index] == 0)
            {
                continue;
            }
            while (*reinterpret_cast<const CargoCategory*>(remainingData.data()) != CargoCategory::null)
            {
                const auto cargoCategory = *reinterpret_cast<const CargoCategory*>(remainingData.data());
                remainingData = remainingData.subspan(sizeof(CargoCategory));
                const auto cargoTypeSpriteOffset = *reinterpret_cast<const uint8_t*>(remainingData.data());
                remainingData = remainingData.subspan(sizeof(uint8_t));

                for (auto cargoType = 0U; cargoType < ObjectManager::getMaxObjects(ObjectType::cargo); ++cargoType)
                {
                    auto* cargoObj = ObjectManager::get<CargoObject>(cargoType);
                    if (cargoObj == nullptr)
                    {
                        continue;
                    }
                    if (cargoObj->cargoCategory != cargoCategory)
                    {
                        continue;
                    }
                    compatibleCargoCategories[index] |= (1U << cargoType);
                    cargoTypeSpriteOffsets[cargoType] = cargoTypeSpriteOffset;
                }
            }
            remainingData = remainingData.subspan(sizeof(uint16_t));
            if (compatibleCargoCategories[index] == 0)
            {
                maxCargo[index] = 0;
            }
            else
            {
                numSimultaneousCargoTypes++;
            }
        }

        for (auto& anim : animation)
        {
            if (anim.type == EmitterAnimationType::none)
            {
                continue;
            }
            ObjectHeader modHeader = *reinterpret_cast<const ObjectHeader*>(remainingData.data());
            remainingData = remainingData.subspan(sizeof(ObjectHeader));
            if (modHeader.getType() != ObjectType::steam)
            {
                continue;
            }
            if (dependencies != nullptr)
            {
                dependencies->required.push_back(modHeader);
            }
            auto res = ObjectManager::findObjectHandle(modHeader);
            if (res.has_value())
            {
                anim.objectId = res->id;
            }
        }

        std::fill(std::begin(compatibleVehicles), std::end(compatibleVehicles), kNullObjectId);
        for (auto i = 0U, index = 0U; i < numCompatibleVehicles; ++i)
        {
            ObjectHeader vehHeader = *reinterpret_cast<const ObjectHeader*>(remainingData.data());
            auto res = ObjectManager::findObjectHandleFuzzy(vehHeader);
            if (res.has_value())
            {
                compatibleVehicles[index++] = res->id;
            }
            remainingData = remainingData.subspan(sizeof(ObjectHeader));
        }

        if (hasFlags(VehicleObjectFlags::rackRail))
        {
            ObjectHeader unkHeader = *reinterpret_cast<const ObjectHeader*>(remainingData.data());
            if (dependencies != nullptr)
            {
                dependencies->required.push_back(unkHeader);
            }
            auto res = ObjectManager::findObjectHandle(unkHeader);
            if (res.has_value())
            {
                rackRailType = res->id;
            }
            remainingData = remainingData.subspan(sizeof(ObjectHeader));
        }

        if (drivingSoundType != DrivingSoundType::none)
        {
            ObjectHeader soundHeader = *reinterpret_cast<const ObjectHeader*>(remainingData.data());
            if (dependencies != nullptr)
            {
                dependencies->required.push_back(soundHeader);
            }
            auto res = ObjectManager::findObjectHandle(soundHeader);
            if (res.has_value())
            {
                sound.friction.soundObjectId = res->id;
            }
            remainingData = remainingData.subspan(sizeof(ObjectHeader));
        }

        const auto startSoundCount = std::min(kMaxStartSounds, numStartSounds & NumStartSounds::kMask);
        for (auto i = 0; i < startSoundCount; ++i)
        {
            ObjectHeader soundHeader = *reinterpret_cast<const ObjectHeader*>(remainingData.data());
            if (dependencies != nullptr)
            {
                dependencies->required.push_back(soundHeader);
            }
            auto res = ObjectManager::findObjectHandle(soundHeader);
            if (res.has_value())
            {
                startSounds[i] = res->id;
            }
            remainingData = remainingData.subspan(sizeof(ObjectHeader));
        }

        auto imgRes = ObjectManager::loadImageTable(remainingData);
        assert(remainingData.size() == imgRes.tableLength);

        auto offset = 0;
        for (auto& bodySprite : bodySprites)
        {
            if (!bodySprite.hasFlags(BodySpriteFlags::hasSprites))
            {
                continue;
            }
            bodySprite.flatImageId = offset + imgRes.imageOffset;
            bodySprite.flatYawAccuracy = getYawAccuracyFlat(bodySprite.numFlatRotationFrames);

            bodySprite.numFramesPerRotation = bodySprite.numAnimationFrames * bodySprite.numCargoFrames * bodySprite.numRollFrames + (bodySprite.hasFlags(BodySpriteFlags::hasBrakingLights) ? 1 : 0);
            const auto numFlatFrames = (bodySprite.numFramesPerRotation * bodySprite.numFlatRotationFrames);
            offset += numFlatFrames / (bodySprite.hasFlags(BodySpriteFlags::rotationalSymmetry) ? 2 : 1);

            if (bodySprite.hasFlags(BodySpriteFlags::hasGentleSprites))
            {
                bodySprite.gentleImageId = offset + imgRes.imageOffset;
                const auto numGentleTransitionFrames = bodySprite.numFramesPerRotation * (4 + 4); // transition frames up/down deg6
                offset += numGentleTransitionFrames / (bodySprite.hasFlags(BodySpriteFlags::rotationalSymmetry) ? 2 : 1);

                bodySprite.slopedYawAccuracy = getYawAccuracySloped(bodySprite.numSlopedRotationFrames);
                const auto numGentleFrames = bodySprite.numFramesPerRotation * bodySprite.numSlopedRotationFrames * 2; // up/down deg12
                offset += numGentleFrames / (bodySprite.hasFlags(BodySpriteFlags::rotationalSymmetry) ? 2 : 1);

                if (bodySprite.hasFlags(BodySpriteFlags::hasSteepSprites))
                {
                    bodySprite.steepImageId = offset + imgRes.imageOffset;
                    const auto numSteepTransitionFrames = bodySprite.numFramesPerRotation * (4 + 4); // transition frames up/down deg18
                    offset += numSteepTransitionFrames / (bodySprite.hasFlags(BodySpriteFlags::rotationalSymmetry) ? 2 : 1);
                    // TODO: add these two together??
                    const auto numSteepFrames = bodySprite.numSlopedRotationFrames * bodySprite.numFramesPerRotation * 2; // up/down deg25
                    offset += numSteepFrames / (bodySprite.hasFlags(BodySpriteFlags::rotationalSymmetry) ? 2 : 1);
                }
            }

            const auto numImages = imgRes.imageOffset + offset - bodySprite.flatImageId;
            if (bodySprite.flatImageId + numImages <= ObjectManager::getTotalNumImages())
            {
                const auto extents = Gfx::getImagesMaxExtent(ImageId(bodySprite.flatImageId), numImages);
                bodySprite.width = extents.width;
                bodySprite.heightNegative = extents.heightNegative;
                bodySprite.heightPositive = extents.heightPositive;
            }
            else
            {
                // This is a bad object! But will keep loading
                Logging::error("Object has too few images for body sprites!");
                bodySprite.flatImageId = ImageId::kIndexUndefined;
                bodySprite.gentleImageId = ImageId::kIndexUndefined;
                bodySprite.steepImageId = ImageId::kIndexUndefined;
                bodySprite.unkImageId = ImageId::kIndexUndefined;
            }
        }

        for (auto& bogieSprite : bogieSprites)
        {
            if (!bogieSprite.hasFlags(BogieSpriteFlags::hasSprites))
            {
                continue;
            }
            bogieSprite.numFramesPerRotation = bogieSprite.numAnimationFrames;
            bogieSprite.flatImageIds = offset + imgRes.imageOffset;

            const auto numFlatFrames = bogieSprite.numFramesPerRotation * 32;
            offset += numFlatFrames / (bogieSprite.hasFlags(BogieSpriteFlags::rotationalSymmetry) ? 2 : 1);

            if (bogieSprite.hasFlags(BogieSpriteFlags::hasGentleSprites))
            {
                bogieSprite.gentleImageIds = offset + imgRes.imageOffset;
                const auto numGentleFrames = bogieSprite.numFramesPerRotation * 32 * 2; // up and down 12 deg
                offset += numGentleFrames / (bogieSprite.hasFlags(BogieSpriteFlags::rotationalSymmetry) ? 2 : 1);

                if (bogieSprite.hasFlags(BogieSpriteFlags::hasSteepSprites))
                {
                    bogieSprite.steepImageIds = offset + imgRes.imageOffset;
                    const auto numSteepFrames = bogieSprite.numFramesPerRotation * 32 * 2; // up and down 25 deg
                    offset += numSteepFrames / (bogieSprite.hasFlags(BogieSpriteFlags::rotationalSymmetry) ? 2 : 1);
                }
            }

            const auto numImages = imgRes.imageOffset + offset - bogieSprite.flatImageIds;
            if (bogieSprite.flatImageIds + numImages <= ObjectManager::getTotalNumImages())
            {
                const auto extents = Gfx::getImagesMaxExtent(ImageId(bogieSprite.flatImageIds), numImages);
                bogieSprite.width = extents.width;
                bogieSprite.heightNegative = extents.heightNegative;
                bogieSprite.heightPositive = extents.heightPositive;
            }
            else
            {
                // This is a bad object! But we will keep loading anyway!
                Logging::error("Object has too few images for bogie sprites!");
                bogieSprite.flatImageIds = ImageId::kIndexUndefined;
                bogieSprite.gentleImageIds = ImageId::kIndexUndefined;
                bogieSprite.steepImageIds = ImageId::kIndexUndefined;
            }
        }

        // Verify we haven't overshot any lengths (See above Rarrr's)
        if (imgRes.imageOffset + offset != ObjectManager::getTotalNumImages())
        {
            // There are some official objects that suffer from this so can't assert on this.
            // TODO: This does not work you can't get a header from a temporary object.
            // This verbose message will only make sense when loading a save/scenario.
            const auto& header = ObjectManager::getHeader(handle);
            std::string objName(header.getName());
            Logging::verbose("Incorrect number of images for object: {}", objName);
        }
    }

    uint8_t getEffectiveVehicleCapacity(const ObjectHeader& header, const uint8_t capacity)
    {
        const auto* capacityOverride = findVehicleCapacityOverride(header);
        if (capacityOverride != nullptr && capacity == capacityOverride->originalCapacity)
        {
            return capacityOverride->correctedCapacity;
        }
        return capacity;
    }

    uint8_t getEffectiveVehicleCapacity(const ObjectHeader& header, const uint8_t capacity, const uint8_t primaryCargoUnitSize, const uint8_t cargoUnitSize)
    {
        const auto* capacityOverride = findVehicleCapacityOverride(header);
        if (capacityOverride == nullptr || cargoUnitSize == 0)
        {
            return capacity;
        }

        const auto convertCapacity = [primaryCargoUnitSize, cargoUnitSize](const uint8_t baseCapacity) {
            const auto convertedCapacity = (static_cast<uint32_t>(primaryCargoUnitSize) * baseCapacity) / cargoUnitSize;
            return static_cast<uint8_t>(std::min(convertedCapacity, 0xFFU));
        };
        const auto convertLegacyCapacity = [primaryCargoUnitSize, cargoUnitSize](const uint8_t baseCapacity) {
            return static_cast<uint8_t>((static_cast<uint32_t>(primaryCargoUnitSize) * baseCapacity) / cargoUnitSize);
        };
        return capacity == convertCapacity(capacityOverride->originalCapacity) || capacity == convertLegacyCapacity(capacityOverride->originalCapacity)
            ? convertCapacity(capacityOverride->correctedCapacity)
            : capacity;
    }

    // 0x004B89FF
    void VehicleObject::unload()
    {
        name = 0;
        trackType = 0;
        for (auto& anim : animation)
        {
            anim.objectId = 0;
        }

        std::fill(std::begin(requiredTrackExtras), std::end(requiredTrackExtras), 0);

        std::fill(std::begin(maxCargo), std::end(maxCargo), 0);
        std::fill(std::begin(compatibleCargoCategories), std::end(compatibleCargoCategories), 0);
        numSimultaneousCargoTypes = 0;

        std::fill(std::begin(cargoTypeSpriteOffsets), std::end(cargoTypeSpriteOffsets), 0);
        std::fill(std::begin(compatibleVehicles), std::end(compatibleVehicles), 0);

        for (auto& bodySprite : bodySprites)
        {
            bodySprite.flatImageId = 0;
            bodySprite.flatYawAccuracy = 0;
            bodySprite.numFramesPerRotation = 0;
            bodySprite.gentleImageId = 0;
            bodySprite.slopedYawAccuracy = 0;
            bodySprite.steepImageId = 0;
            bodySprite.unkImageId = 0;
            bodySprite.width = 0;
            bodySprite.heightNegative = 0;
            bodySprite.heightPositive = 0;
        }

        for (auto& bogieSprite : bogieSprites)
        {
            bogieSprite.flatImageIds = 0;
            bogieSprite.gentleImageIds = 0;
            bogieSprite.steepImageIds = 0;
            bogieSprite.width = 0;
            bogieSprite.heightNegative = 0;
            bogieSprite.heightPositive = 0;
            bogieSprite.numFramesPerRotation = 0;
        }

        rackRailType = 0;
        sound.simpleMotor.soundObjectId = 0;

        std::fill(std::begin(startSounds), std::end(startSounds), 0);
    }

    // 0x004B9780
    uint32_t VehicleObject::getLength() const
    {
        auto length = 0;
        for (auto i = 0; i < numCarComponents; ++i)
        {
            if (carComponents[i].bodySpriteInd == 0xFF)
            {
                continue;
            }

            auto unk = carComponents[i].bodySpriteInd & (VehicleObject::kMaxBodySprites - 1);
            length += bodySprites[unk].halfLength * 2;
        }
        return length;
    }
}
