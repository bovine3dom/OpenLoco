#include "GameCommands/Undo.h"

#include "CargoDist/Simulation.h"
#include "Entities/EntityManager.h"
#include "Entities/EntityTweener.h"
#include "GameCommands/Airports/CreateAirport.h"
#include "GameState.h"
#include "Graphics/Gfx.h"
#include "Map/TileManager.h"
#include "Map/Track/TrackOverlayAudits.h"
#include "Network/Network.h"
#include "Objects/AirportObject.h"
#include "Objects/IndustryObject.h"
#include "Objects/ObjectManager.h"
#include "SceneManager.h"
#include "Ui/WindowManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/SharedOrderManager.h"
#include "World/CompanyManager.h"
#include "World/StationManager.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <vector>

namespace OpenLoco::GameCommands::Undo
{
    using namespace World;

    struct ElementSnapshot
    {
        ElementType type;
        std::array<uint8_t, kTileElementSize> data;

        bool operator==(const ElementSnapshot&) const = default;
    };

    struct TilePatch
    {
        TilePos2 pos;
        std::vector<ElementSnapshot> before;
        std::vector<ElementSnapshot> after;
    };

    struct BytePatch
    {
        uint32_t offset;
        std::vector<uint8_t> before;
        std::vector<uint8_t> after;
    };

    struct StateGuard
    {
        uint32_t offset;
        std::vector<uint8_t> after;
    };

    struct SharedOrderPatch
    {
        Vehicles::SharedOrderManager::State before;
        Vehicles::SharedOrderManager::State after;
    };

    struct PendingTransaction
    {
        CompanyId company;
        bool captureEntities;
        std::vector<uint8_t> state;
        std::vector<TilePatch> tiles;
        Vehicles::SharedOrderManager::State sharedOrders;
    };

    struct HistoryEntry
    {
        struct Payment
        {
            CompanyId company;
            currency32_t cost;
            ExpenditureType expenditureType;
            Pos3 position;
        };

        std::vector<Payment> payments;
        std::vector<BytePatch> state;
        std::vector<StateGuard> stateGuards;
        std::vector<StationId> createdStations;
        std::vector<TilePatch> tiles;
        std::optional<SharedOrderPatch> sharedOrders;
    };

    struct ByteChange
    {
        uint8_t before;
        uint8_t after;
    };

    struct StateByteMask
    {
        // Derived state may need restoring without making normal updates invalidate undo.
        uint8_t capture;
        uint8_t validate;
    };

    struct HistoryGroup
    {
        HistoryEntry history{};
        std::map<uint32_t, ByteChange> state;
        std::map<uint32_t, TilePatch> tiles;
    };

    static std::optional<PendingTransaction> _pending;
    static std::optional<HistoryEntry> _history;
    static std::optional<HistoryGroup> _group;
    static uint32_t _groupDepth;

    static bool isMapCommand(const GameCommand command)
    {
        return command != GameCommand::vehicleCreate
            && command != GameCommand::vehicleClone
            && command != GameCommand::vehicleOrderShare;
    }

    static bool isUndoableCommand(const GameCommand command)
    {
        switch (command)
        {
            case GameCommand::vehicleCreate:
            case GameCommand::vehicleClone:
            case GameCommand::vehicleOrderShare:
            case GameCommand::createTrack:
            case GameCommand::createSignal:
            case GameCommand::createTrainStation:
            case GameCommand::createTrackMod:
            case GameCommand::createRoad:
            case GameCommand::createRoadMod:
            case GameCommand::createRoadStation:
            case GameCommand::createTree:
            case GameCommand::removeTree:
            case GameCommand::changeLandMaterial:
            case GameCommand::raiseLand:
            case GameCommand::lowerLand:
            case GameCommand::lowerRaiseLandMountain:
            case GameCommand::raiseWater:
            case GameCommand::lowerWater:
            case GameCommand::createWall:
            case GameCommand::removeWall:
            case GameCommand::clearLand:
            case GameCommand::createBuilding:
            case GameCommand::createIndustry:
            case GameCommand::buildCompanyHeadquarters:
            case GameCommand::createAirport:
            case GameCommand::createPort:
                return true;

            default:
                return false;
        }
    }

    static size_t getStateSize()
    {
        const auto* begin = reinterpret_cast<const uint8_t*>(&getGameState());
        const auto* end = reinterpret_cast<const uint8_t*>(&getGameState().tileState);
        return static_cast<size_t>(end - begin);
    }

    static StateByteMask getStateByteMask(const size_t offset)
    {
        constexpr StateByteMask kDefaultMask{ 0xFF, 0xFF };
        constexpr StateByteMask kIgnoredMask{ 0, 0 };
        const auto& gameState = getGameState();
        const auto* state = reinterpret_cast<const uint8_t*>(&gameState);
        const auto getOffset = [state](const auto* member) {
            return static_cast<size_t>(reinterpret_cast<const uint8_t*>(member) - state);
        };

        if (offset >= getOffset(&gameState.rng) && offset < getOffset(&gameState.flags))
        {
            return kIgnoredMask;
        }

        const auto companiesBegin = getOffset(&gameState.companies);
        const auto companiesEnd = companiesBegin + sizeof(gameState.companies);
        if (offset >= companiesBegin && offset < companiesEnd)
        {
            static const auto masks = [=] {
                std::array<StateByteMask, sizeof(Company)> result;
                result.fill(kDefaultMask);
                const auto& company = getGameState().companies[0];
                const auto* begin = reinterpret_cast<const uint8_t*>(&company);
                const auto transientBegin = reinterpret_cast<const uint8_t*>(&company.activeEmotions) - begin;
                const auto transientEnd = reinterpret_cast<const uint8_t*>(&company.ownerStatus) - begin;
                std::fill(result.begin() + transientBegin, result.begin() + transientEnd, kIgnoredMask);
                return result;
            }();
            return masks[(offset - companiesBegin) % sizeof(Company)];
        }

        const auto stationsBegin = getOffset(&gameState.stations);
        const auto stationsEnd = stationsBegin + sizeof(gameState.stations);
        if (offset >= stationsBegin && offset < stationsEnd)
        {
            static const auto masks = [=] {
                std::array<StateByteMask, sizeof(Station)> result;
                result.fill(kDefaultMask);
                const auto& station = getGameState().stations[0];
                const auto* begin = reinterpret_cast<const uint8_t*>(&station);
                const auto ignoreValidation = [&](const auto& member) {
                    const auto memberOffset = reinterpret_cast<const uint8_t*>(&member) - begin;
                    for (size_t i = 0; i < sizeof(member); ++i)
                    {
                        result[memberOffset + i].validate = 0;
                    }
                };
                ignoreValidation(station.labelFrame);
                ignoreValidation(station.noTilesTimeout);
                for (const auto& cargo : station.cargoStats)
                {
                    const auto flagsOffset = reinterpret_cast<const uint8_t*>(&cargo.flags) - begin;
                    result[flagsOffset].validate &= ~enumValue(StationCargoStatsFlags::acceptedForConsumer);
                    ignoreValidation(cargo.industryId);
                }
                ignoreValidation(station.var_3B0);
                ignoreValidation(station.var_3B1);
                return result;
            }();
            return masks[(offset - stationsBegin) % sizeof(Station)];
        }

        return kDefaultMask;
    }

    static bool stateMatches(const uint8_t* state, const uint32_t offset, const std::span<const uint8_t> expected)
    {
        for (size_t i = 0; i < expected.size(); ++i)
        {
            const auto mask = getStateByteMask(offset + i).validate;
            if (((state[offset + i] ^ expected[i]) & mask) != 0)
            {
                return false;
            }
        }
        return true;
    }

    static ElementSnapshot captureElement(const TileElementEntry& entry)
    {
        ElementSnapshot result{ entry.type(), {} };
        const auto data = TileManager::resolveEntry(&entry).rawData();
        std::ranges::copy(data, result.data.begin());
        return result;
    }

    static std::vector<ElementSnapshot> captureTile(const TilePos2& pos)
    {
        std::vector<ElementSnapshot> result;
        auto tile = TileManager::get(pos);
        result.reserve(tile.size());
        for (const auto& entry : tile)
        {
            result.push_back(captureElement(entry));
        }
        return result;
    }

    static uint8_t getTileByteValidationMask(const ElementType type, const size_t index)
    {
        // The surface update timer advances independently of construction.
        return type == ElementType::surface && index == 5 ? 0x1F : 0xFF;
    }

    static bool tileMatches(const TilePos2& pos, const std::span<const ElementSnapshot> expected)
    {
        auto tile = TileManager::get(pos);
        if (tile.size() != expected.size())
        {
            return false;
        }

        size_t index = 0;
        for (const auto& entry : tile)
        {
            const auto actual = captureElement(entry);
            const auto& expectedElement = expected[index++];
            if (actual.type != expectedElement.type)
            {
                return false;
            }
            for (size_t i = 0; i < actual.data.size(); ++i)
            {
                if (((actual.data[i] ^ expectedElement.data[i]) & getTileByteValidationMask(actual.type, i)) != 0)
                {
                    return false;
                }
            }
        }
        return true;
    }

    static std::vector<BytePatch> createStatePatches(const std::vector<uint8_t>& before, const bool captureEntities)
    {
        const auto& gameState = getGameState();
        const auto* after = reinterpret_cast<const uint8_t*>(&gameState);
        const auto getOffset = [after](const auto* member) {
            return static_cast<size_t>(reinterpret_cast<const uint8_t*>(member) - after);
        };
        const auto entityListsBegin = getOffset(&gameState.entityListHeads);
        const auto entityListsEnd = getOffset(&gameState.currencyMultiplicationFactor);
        const auto entitiesBegin = getOffset(&gameState.entities);
        const auto entitiesEnd = getOffset(&gameState.animations);
        const auto shouldIgnore = [=](const size_t offset) {
            return !captureEntities
                && ((offset >= entityListsBegin && offset < entityListsEnd)
                    || (offset >= entitiesBegin && offset < entitiesEnd));
        };
        const auto hasChanged = [&](const size_t offset) {
            return !shouldIgnore(offset)
                && ((before[offset] ^ after[offset]) & getStateByteMask(offset).capture) != 0;
        };

        std::vector<BytePatch> result;
        constexpr size_t kScanBlockSize = 256;
        for (size_t blockStart = 0; blockStart < before.size(); blockStart += kScanBlockSize)
        {
            const auto blockEnd = std::min(blockStart + kScanBlockSize, before.size());
            if (std::memcmp(before.data() + blockStart, after + blockStart, blockEnd - blockStart) == 0)
            {
                continue;
            }

            auto offset = blockStart;
            while (offset < blockEnd)
            {
                while (offset < blockEnd && !hasChanged(offset))
                {
                    ++offset;
                }
                const auto start = offset;
                while (offset < blockEnd && hasChanged(offset))
                {
                    ++offset;
                }
                if (start != offset)
                {
                    result.push_back({
                        static_cast<uint32_t>(start),
                        std::vector<uint8_t>(before.begin() + start, before.begin() + offset),
                        std::vector<uint8_t>(after + start, after + offset),
                    });
                }
            }
        }
        return result;
    }

    static void createStateGuards(const PendingTransaction& pending, HistoryEntry& history)
    {
        const auto& gameState = getGameState();
        const auto* state = reinterpret_cast<const uint8_t*>(&gameState);
        const auto addGuard = [&](const auto& object) {
            const auto* begin = reinterpret_cast<const uint8_t*>(&object);
            history.stateGuards.push_back({
                static_cast<uint32_t>(begin - state),
                std::vector<uint8_t>(begin, begin + sizeof(object)),
            });
        };

        for (const auto& station : gameState.stations)
        {
            const auto offset = reinterpret_cast<const uint8_t*>(&station) - state;
            StringId previousName;
            std::memcpy(&previousName, pending.state.data() + offset, sizeof(previousName));
            if (previousName == StringIds::null && !station.empty())
            {
                addGuard(station);
                history.createdStations.push_back(station.id());
            }
        }

        for (const auto& industry : gameState.industries)
        {
            const auto offset = reinterpret_cast<const uint8_t*>(&industry) - state;
            StringId previousName;
            std::memcpy(&previousName, pending.state.data() + offset, sizeof(previousName));
            if (previousName == StringIds::null && !industry.empty())
            {
                addGuard(industry);
            }
        }

        if (pending.captureEntities)
        {
            for (const auto& entity : gameState.entities)
            {
                const auto offset = reinterpret_cast<const uint8_t*>(&entity) - state;
                if (std::memcmp(pending.state.data() + offset, &entity, sizeof(entity)) != 0)
                {
                    addGuard(entity);
                }
            }
        }
    }

    static std::vector<TilePatch> createTilePatches(std::vector<TilePatch> patches)
    {
        for (auto& patch : patches)
        {
            patch.after = captureTile(patch.pos);
        }
        std::erase_if(patches, [](const auto& patch) { return patch.before == patch.after; });
        return patches;
    }

    struct AffectedArea
    {
        int32_t x1;
        int32_t y1;
        int32_t x2;
        int32_t y2;
    };

    static AffectedArea getAffectedArea(const GameCommand command, const CompanyId company, const registers& regs)
    {
        auto pointA = Pos2{ regs.ax, regs.cx };
        auto pointB = pointA;
        auto padding = 4 * kTileSize;
        switch (command)
        {
            case GameCommand::createTrackMod:
            case GameCommand::createRoadMod:
                if (((regs.ebp >> 16) & 0xFF) != 0)
                {
                    return { 0, 0, (kMapColumns - 1) * kTileSize, (kMapRows - 1) * kTileSize };
                }
                break;

            case GameCommand::raiseLand:
            case GameCommand::lowerLand:
            case GameCommand::clearLand:
                pointA = { regs.dx, regs.bp };
                pointB = { static_cast<int16_t>(regs.edx >> 16), static_cast<int16_t>(regs.ebp >> 16) };
                padding = 8 * kTileSize;
                break;

            case GameCommand::lowerRaiseLandMountain:
                pointA = { regs.dx, regs.bp };
                pointB = { static_cast<int16_t>(regs.edx >> 16), static_cast<int16_t>(regs.ebp >> 16) };
                padding = 32 * kTileSize;
                break;

            case GameCommand::changeLandMaterial:
            case GameCommand::raiseWater:
            case GameCommand::lowerWater:
                pointB = { regs.di, regs.bp };
                padding = 8 * kTileSize;
                break;

            case GameCommand::createBuilding:
            case GameCommand::createPort:
                padding = 16 * kTileSize;
                break;

            case GameCommand::createIndustry:
            {
                const auto* object = ObjectManager::get<IndustryObject>(regs.dl & 0x7F);
                const auto maxBuildings = object == nullptr ? 32 : std::min<uint8_t>(object->maxNumBuildings, 32);
                padding = (2 * maxBuildings + 18) * kTileSize;
                break;
            }

            case GameCommand::createAirport:
            {
                const AirportPlacementArgs args(regs);
                const auto* object = ObjectManager::get<AirportObject>(args.type);
                if (object != nullptr)
                {
                    const auto [minExtent, maxExtent] = object->getAirportExtents(toTileSpace(args.pos), args.rotation);
                    pointA = toWorldSpace(minExtent);
                    pointB = toWorldSpace(maxExtent);
                    padding = 2 * kTileSize;
                }
                else
                {
                    padding = 16 * kTileSize;
                }
                break;
            }

            case GameCommand::buildCompanyHeadquarters:
            {
                const auto* owner = CompanyManager::get(company);
                if (owner != nullptr && owner->headquartersX != -1)
                {
                    pointB = { owner->headquartersX, owner->headquartersY };
                }
                padding = 4 * kTileSize;
                break;
            }

            case GameCommand::createTree:
            case GameCommand::removeTree:
            case GameCommand::createWall:
            case GameCommand::removeWall:
                padding = 2 * kTileSize;
                break;

            default:
                break;
        }
        return { std::min(pointA.x, pointB.x) - padding,
                 std::min(pointA.y, pointB.y) - padding,
                 std::max(pointA.x, pointB.x) + padding,
                 std::max(pointA.y, pointB.y) + padding };
    }

    static std::vector<TilePatch> captureAffectedTiles(const GameCommand command, const CompanyId company, const registers& regs)
    {
        const auto area = getAffectedArea(command, company, regs);
        const auto minX = static_cast<tile_coord_t>(std::clamp(area.x1 / kTileSize, 0, static_cast<int32_t>(kMapColumns - 1)));
        const auto minY = static_cast<tile_coord_t>(std::clamp(area.y1 / kTileSize, 0, static_cast<int32_t>(kMapRows - 1)));
        const auto maxX = static_cast<tile_coord_t>(std::clamp(area.x2 / kTileSize, 0, static_cast<int32_t>(kMapColumns - 1)));
        const auto maxY = static_cast<tile_coord_t>(std::clamp(area.y2 / kTileSize, 0, static_cast<int32_t>(kMapRows - 1)));

        std::vector<TilePatch> result;
        result.reserve(static_cast<size_t>(maxX - minX + 1) * (maxY - minY + 1));
        for (auto y = minY; y <= maxY; ++y)
        {
            for (auto x = minX; x <= maxX; ++x)
            {
                const TilePos2 pos{ x, y };
                result.push_back({ pos, captureTile(pos), {} });
            }
        }
        return result;
    }

    static bool isHistoryValid()
    {
        if (!_history.has_value() || Network::isConnected())
        {
            return false;
        }

        const auto* state = reinterpret_cast<const uint8_t*>(&getGameState());
        for (const auto& guard : _history->stateGuards)
        {
            if (!stateMatches(state, guard.offset, guard.after))
            {
                return false;
            }
        }
        for (const auto& patch : _history->state)
        {
            if (!stateMatches(state, patch.offset, patch.after))
            {
                return false;
            }
        }
        if (_history->sharedOrders.has_value()
            && (!Vehicles::SharedOrderManager::validateState(_history->sharedOrders->before)
                || !Vehicles::SharedOrderManager::validateState(_history->sharedOrders->after)
                || Vehicles::SharedOrderManager::captureState() != _history->sharedOrders->after))
        {
            return false;
        }
        for (const auto& patch : _history->tiles)
        {
            for (auto* entity : EntityManager::EntityTileList(toWorldSpace(patch.pos)))
            {
                if (entity->baseType == EntityBaseType::vehicle)
                {
                    return false;
                }
            }
        }
        return std::ranges::all_of(_history->tiles, [](const auto& patch) {
            return tileMatches(patch.pos, patch.after);
        });
    }

    static bool restoreTiles(const std::vector<TilePatch>& patches)
    {
        size_t currentElements = 0;
        size_t restoredElements = 0;
        for (const auto& patch : patches)
        {
            if (patch.before.empty() || patch.before.front().type != ElementType::surface)
            {
                return false;
            }
            currentElements += TileManager::get(patch.pos).size() - 1;
            restoredElements += patch.before.size() - 1;
        }
        if (restoredElements > currentElements)
        {
            TileManager::reorganise();
            if (restoredElements - currentElements > TileManager::numFreeElements())
            {
                return false;
            }
        }

        Track::TrackOverlayAudits::invalidateAudit();
        for (const auto& patch : patches)
        {
            while (TileManager::get(patch.pos).size() > 1)
            {
                auto tile = TileManager::get(patch.pos);
                TileManager::removeElement(*tile[tile.size() - 1]);
            }
            auto tile = TileManager::get(patch.pos);
            auto& surface = TileManager::resolveEntry(tile.surfaceEntry());
            auto surfaceData = surface.rawData();
            const auto updateTimer = surfaceData[5] & 0xE0;
            std::ranges::copy(patch.before.front().data, surfaceData.begin());
            if (((patch.before.front().data[5] ^ patch.after.front().data[5]) & 0xE0) == 0)
            {
                surfaceData[5] = (surfaceData[5] & 0x1F) | updateTimer;
            }
        }

        for (const auto& patch : patches)
        {
            const auto worldPos = toWorldSpace(patch.pos);
            for (const auto& element : std::span<const ElementSnapshot>(patch.before).subspan(1))
            {
                auto* entry = TileManager::insertElement(element.type, worldPos, element.data[2], element.data[1] & 0xF);
                if (entry == nullptr)
                {
                    return false;
                }
                auto& inserted = TileManager::resolveEntry(entry);
                std::ranges::copy(element.data, inserted.rawData().begin());
            }
            TileManager::mapInvalidateTileFull(worldPos);
        }
        return true;
    }

    static void addToGroup(HistoryEntry&& history)
    {
        if (!_group.has_value())
        {
            _group.emplace();
        }

        auto& group = *_group;
        group.history.payments.insert(group.history.payments.end(), history.payments.begin(), history.payments.end());

        for (const auto& patch : history.state)
        {
            for (size_t i = 0; i < patch.before.size(); ++i)
            {
                const auto offset = patch.offset + static_cast<uint32_t>(i);
                auto [it, inserted] = group.state.try_emplace(offset, ByteChange{ patch.before[i], patch.after[i] });
                if (!inserted)
                {
                    it->second.after = patch.after[i];
                }
            }
        }

        for (auto& guard : history.stateGuards)
        {
            const auto existing = std::ranges::find(group.history.stateGuards, guard.offset, &StateGuard::offset);
            if (existing == group.history.stateGuards.end())
            {
                group.history.stateGuards.push_back(std::move(guard));
            }
        }

        for (const auto station : history.createdStations)
        {
            if (std::ranges::find(group.history.createdStations, station) == group.history.createdStations.end())
            {
                group.history.createdStations.push_back(station);
            }
        }

        for (auto& patch : history.tiles)
        {
            const auto key = static_cast<uint32_t>(patch.pos.y) * kMapColumns + patch.pos.x;
            auto [it, inserted] = group.tiles.try_emplace(key, std::move(patch));
            if (!inserted)
            {
                it->second.after = std::move(patch.after);
            }
        }

        if (history.sharedOrders.has_value())
        {
            if (group.history.sharedOrders.has_value())
            {
                group.history.sharedOrders->after = std::move(history.sharedOrders->after);
            }
            else
            {
                group.history.sharedOrders = std::move(history.sharedOrders);
            }
        }
    }

    static void finishGroup()
    {
        if (!_group.has_value())
        {
            return;
        }

        auto group = std::move(*_group);
        _group.reset();

        for (const auto& [offset, change] : group.state)
        {
            if (change.before == change.after)
            {
                continue;
            }

            if (group.history.state.empty()
                || offset != group.history.state.back().offset + group.history.state.back().before.size())
            {
                group.history.state.push_back({ offset, {}, {} });
            }
            group.history.state.back().before.push_back(change.before);
            group.history.state.back().after.push_back(change.after);
        }

        const auto* state = reinterpret_cast<const uint8_t*>(&getGameState());
        for (auto& guard : group.history.stateGuards)
        {
            guard.after.assign(state + guard.offset, state + guard.offset + guard.after.size());
        }

        for (auto& [_, patch] : group.tiles)
        {
            if (patch.before != patch.after)
            {
                group.history.tiles.push_back(std::move(patch));
            }
        }
        if (group.history.sharedOrders.has_value()
            && group.history.sharedOrders->before == group.history.sharedOrders->after)
        {
            group.history.sharedOrders.reset();
        }

        _history = std::move(group.history);
        Ui::WindowManager::invalidate(Ui::WindowType::topToolbar);
    }

    static HistoryEntry capturePending()
    {
        HistoryEntry history{};
        history.state = createStatePatches(_pending->state, _pending->captureEntities);
        createStateGuards(*_pending, history);
        history.tiles = createTilePatches(std::move(_pending->tiles));
        auto sharedOrdersAfter = Vehicles::SharedOrderManager::captureState();
        if (_pending->sharedOrders != sharedOrdersAfter)
        {
            history.sharedOrders = SharedOrderPatch{ std::move(_pending->sharedOrders), std::move(sharedOrdersAfter) };
        }
        _pending.reset();
        return history;
    }

    void prepare(const GameCommand command, const CompanyId company, const registers& regs, const uint8_t flags)
    {
        _pending.reset();
        if (!isUndoableCommand(command)
            || (flags & (Flags::ghost | Flags::aiAllocated | Flags::noPayment)) != 0
            || Network::isConnected()
            || (!SceneManager::isEditorMode() && company != CompanyManager::getControllingId()))
        {
            return;
        }

        PendingTransaction transaction{};
        transaction.company = company;
        transaction.captureEntities = !isMapCommand(command);
        transaction.state.resize(getStateSize());
        std::memcpy(transaction.state.data(), &getGameState(), transaction.state.size());
        transaction.sharedOrders = Vehicles::SharedOrderManager::captureState();
        if (isMapCommand(command))
        {
            transaction.tiles = captureAffectedTiles(command, company, regs);
        }
        _pending = std::move(transaction);
    }

    void commit(const currency32_t cost, const ExpenditureType expenditureType, const Pos3& position)
    {
        if (!_pending.has_value())
        {
            return;
        }

        const auto company = _pending->company;
        auto history = capturePending();
        history.payments.push_back({ company, cost, expenditureType, position });
        if (_groupDepth != 0)
        {
            addToGroup(std::move(history));
        }
        else
        {
            _history = std::move(history);
            Ui::WindowManager::invalidate(Ui::WindowType::topToolbar);
        }
    }

    void cancel()
    {
        if (_groupDepth != 0 && _pending.has_value())
        {
            auto history = capturePending();
            if (!history.state.empty() || !history.stateGuards.empty() || !history.tiles.empty() || history.sharedOrders.has_value())
            {
                addToGroup(std::move(history));
            }
            return;
        }
        _pending.reset();
    }

    Group::Group()
    {
        if (_groupDepth++ == 0)
        {
            _group.reset();
        }
    }

    Group::~Group()
    {
        if (_groupDepth == 0 || --_groupDepth != 0)
        {
            return;
        }
        finishGroup();
    }

    void clear()
    {
        _pending.reset();
        _group.reset();
        _groupDepth = 0;
        _history.reset();
        Ui::WindowManager::invalidate(Ui::WindowType::topToolbar);
    }

    bool isAvailable()
    {
        return _history.has_value() && !Network::isConnected();
    }

    Result apply()
    {
        if (!_history.has_value())
        {
            return Result::unavailable;
        }
        if (!isHistoryValid())
        {
            clear();
            return Result::stateChanged;
        }

        auto history = std::move(*_history);
        _history.reset();
        if (!restoreTiles(history.tiles))
        {
            clear();
            return Result::stateChanged;
        }

        auto* state = reinterpret_cast<uint8_t*>(&getGameState());
        const auto stationsBegin = static_cast<size_t>(reinterpret_cast<uint8_t*>(&getGameState().stations) - state);
        const auto stationsEnd = stationsBegin + sizeof(getGameState().stations);
        const auto stationsChanged = std::ranges::any_of(history.state, [=](const auto& patch) {
            return patch.offset < stationsEnd && patch.offset + patch.before.size() > stationsBegin;
        });
        for (const auto& patch : history.state)
        {
            std::memcpy(state + patch.offset, patch.before.data(), patch.before.size());
        }
        Vehicles::OrderManager::clearNumDisplayFrames();
        bool sharedOrdersRestored = true;
        if (history.sharedOrders.has_value())
        {
            sharedOrdersRestored = Vehicles::SharedOrderManager::restoreState(history.sharedOrders->before);
        }
        EntityManager::resetSpatialIndex();
        EntityTweener::get().reset();
        for (const auto station : history.createdStations)
        {
            CargoDist::removeStation(station);
        }
        CargoDist::markServicesDirty();
        if (stationsChanged)
        {
            StationManager::updateLabels();
        }

        int64_t moneyEffectAmount = 0;
        Pos3 moneyEffectPosition{};
        for (const auto& payment : history.payments)
        {
            CompanyManager::applyPaymentToCompany(payment.company, -payment.cost, payment.expenditureType);
            if (payment.cost != 0 && payment.company == CompanyManager::getControllingId())
            {
                moneyEffectAmount -= payment.cost;
                moneyEffectPosition = payment.position;
            }
        }
        while (moneyEffectAmount != 0)
        {
            constexpr auto kMaxEffect = std::numeric_limits<currency32_t>::max();
            const auto amount = static_cast<currency32_t>(std::clamp<int64_t>(moneyEffectAmount, -kMaxEffect, kMaxEffect));
            CompanyManager::spendMoneyEffect(moneyEffectPosition + Pos3{ 0, 0, 24 }, CompanyManager::getControllingId(), amount);
            moneyEffectAmount -= amount;
        }
        Gfx::invalidateScreen();
        Ui::WindowManager::invalidateAllWindowsAfterInput();
        if (!sharedOrdersRestored)
        {
            clear();
            return Result::stateChanged;
        }
        return Result::success;
    }
}
