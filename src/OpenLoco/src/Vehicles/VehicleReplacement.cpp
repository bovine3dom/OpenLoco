#include "Vehicles/VehicleReplacement.h"
#include "Entities/EntityManager.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/CloneVehicle.h"
#include "GameCommands/Vehicles/VehiclePlace.h"
#include "GameCommands/Vehicles/VehicleSell.h"
#include "GameState.h"
#include "Localisation/StringIds.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include <algorithm>
#include <ranges>

namespace OpenLoco::Vehicles::VehicleReplacement
{
    static State _state;

    static VehicleHead* getHead(const EntityId id)
    {
        auto* vehicle = EntityManager::get<VehicleBase>(id);
        return vehicle != nullptr && vehicle->isVehicleHead() ? vehicle->asVehicleHead() : nullptr;
    }

    void reset()
    {
        _state = {};
    }

    void remove(const EntityId vehicle)
    {
        std::erase_if(_state.requests, [vehicle](const auto& request) { return request.target == vehicle || request.source == vehicle; });
    }

    bool schedule(const EntityId source)
    {
        const auto members = SharedOrderManager::getMembers(source);
        if (members.size() < 2 || getHead(source) == nullptr)
        {
            return false;
        }

        for (const auto target : members)
        {
            if (target == source)
            {
                continue;
            }
            remove(target);
            _state.requests.push_back({ target, source });
        }
        std::ranges::sort(_state.requests, {}, &Request::target);
        return true;
    }
    // Attempts to place the clone of a replaced vehicle. If the target block is
    // currently occupied by another vehicle the placement is deferred and retried
    // each tick until the block clears. No error window is shown for these
    // automatic placements. The clone inherits whether the replaced vehicle was
    // commanded to stop, so a running replacement continues moving.
    static void placeOrDefer(const GameCommands::VehiclePlacementArgs& placeArgs, const bool startVehicle, VehicleHead& newHead)
    {
        const auto flags = GameCommands::Flags::apply | GameCommands::Flags::noErrorWindow;
        if (GameCommands::doCommand(placeArgs, flags) == GameCommands::kFailure)
        {
            const auto error = GameCommands::getErrorText();
            if (error == StringIds::vehicle_approaching_or_in_the_way || error == StringIds::not_enough_space_or_vehicle_in_the_way)
            {
                _state.pendingPlacements.push_back({ placeArgs, startVehicle });
                return;
            }
            VehicleManager::deleteTrain(newHead);
            return;
        }
        if (startVehicle)
        {
            newHead.vehicleFlags &= ~VehicleFlags::commandStop;
        }
    }

    bool tryReplace(VehicleHead& head)
    {
        const auto targetId = head.id;
        const auto it = std::ranges::find(_state.requests, targetId, &Request::target);
        if (it == _state.requests.end())
        {
            return false;
        }

        auto* source = getHead(it->source);
        if (source == nullptr || source->owner != head.owner || !SharedOrderManager::isShared(source->id)
            || SharedOrderManager::getGroupId(source->id) != SharedOrderManager::getGroupId(targetId))
        {
            remove(targetId);
            return false;
        }
        if (!head.canBeModified())
        {
            return false;
        }
        if (head.mode != TransportMode::rail && head.mode != TransportMode::road)
        {
            return false;
        }

        const auto previousUpdatingCompany = GameCommands::getUpdatingCompanyId();
        GameCommands::setUpdatingCompanyId(head.owner);

        const auto startVehicle = !head.hasVehicleFlags(VehicleFlags::commandStop);

        GameCommands::VehicleCloneArgs cloneArgs{};
        cloneArgs.vehicleHeadId = source->id;
        cloneArgs.shareOrders = true;
        GameCommands::VehicleSellArgs sellArgs{};
        sellArgs.car = targetId;
        GameCommands::VehiclePlacementArgs placeArgs{};
        placeArgs.pos = head.getTrackLoc();
        placeArgs.trackAndDirection = head.trackAndDirection.track._data;
        placeArgs.trackProgress = head.subPosition;
        bool success = GameCommands::doCommand(cloneArgs, 0) != GameCommands::kFailure
            && GameCommands::doCommand(sellArgs, 0) != GameCommands::kFailure
            && GameCommands::doCommand(cloneArgs, GameCommands::Flags::apply) != GameCommands::kFailure;
        if (success)
        {
            const auto* newVehicle = EntityManager::get<VehicleBase>(GameCommands::getLegacyReturnState().lastCreatedVehicleId);
            auto* newHead = newVehicle == nullptr ? nullptr : EntityManager::get<VehicleHead>(newVehicle->getHead());
            success = newHead != nullptr && GameCommands::doCommand(sellArgs, GameCommands::Flags::apply) != GameCommands::kFailure;
            if (success)
            {
                placeArgs.head = newHead->id;
                placeOrDefer(placeArgs, startVehicle, *newHead);
            }
        }
        GameCommands::setUpdatingCompanyId(previousUpdatingCompany);

        if (!success)
        {
            return false;
        }
        remove(targetId);
        return true;
    }

    // Retries placing any deferred replacement vehicles whose target block has
    // since cleared. Pending placements whose vehicle no longer exists, or that
    // cannot be placed for a non-transient reason, are dropped. No error window
    // is shown for these automatic retries.
    void tick()
    {
        if (_state.pendingPlacements.empty())
        {
            return;
        }
        const auto previousUpdatingCompany = GameCommands::getUpdatingCompanyId();
        const auto flags = GameCommands::Flags::apply | GameCommands::Flags::noErrorWindow;
        std::erase_if(_state.pendingPlacements, [&](const auto& placement) {
            auto* head = getHead(placement.args.head);
            if (head == nullptr)
            {
                return true;
            }
            GameCommands::setUpdatingCompanyId(head->owner);
            if (GameCommands::doCommand(placement.args, flags) == GameCommands::kFailure)
            {
                const auto error = GameCommands::getErrorText();
                const auto isTransient = error == StringIds::vehicle_approaching_or_in_the_way
                    || error == StringIds::not_enough_space_or_vehicle_in_the_way;
                if (isTransient)
                {
                    return false;
                }
                VehicleManager::deleteTrain(*head);
                return true;
            }
            if (placement.start)
            {
                head->vehicleFlags &= ~VehicleFlags::commandStop;
            }
            return true;
        });
        GameCommands::setUpdatingCompanyId(previousUpdatingCompany);
    }

    State captureState()
    {
        return _state;
    }

    bool validateState(const State& state)
    {
        return std::ranges::is_sorted(state.requests, {}, &Request::target)
            && std::ranges::all_of(state.requests, [](const auto& request) {
                   return request.target != EntityId::null && request.source != EntityId::null && request.target != request.source;
               })
            && std::ranges::adjacent_find(state.requests, {}, &Request::target) == state.requests.end() && std::ranges::all_of(state.pendingPlacements, [](const auto& placement) { return placement.args.head != EntityId::null; }) && std::ranges::adjacent_find(state.pendingPlacements, {}, [](const auto& placement) { return placement.args.head; }) == state.pendingPlacements.end();
    }

    bool validateState(const State& state, const GameState& gameState)
    {
        const auto validHead = [&gameState](const EntityId id) {
            if (enumValue(id) >= std::size(gameState.entities))
            {
                return false;
            }
            const auto* vehicle = gameState.entities[enumValue(id)].asBase<VehicleBase>();
            return vehicle != nullptr && vehicle->isVehicleHead() && vehicle->id == id;
        };
        return validateState(state) && std::ranges::all_of(state.requests, [&validHead](const auto& request) {
                   return validHead(request.target) && validHead(request.source);
               })
            && std::ranges::all_of(state.pendingPlacements, [&validHead](const auto& placement) { return validHead(placement.args.head); });
    }

    bool restoreState(const State& state)
    {
        if (!validateState(state))
        {
            return false;
        }
        _state = state;
        return true;
    }
}
