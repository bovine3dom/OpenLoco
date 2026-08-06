#include "Vehicles/RoutingManager.h"
#include "GameState.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Vehicles/PathSignals.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleHead.h"
#include <algorithm>
#include <bit>
#include <deque>

namespace OpenLoco::Vehicles::RoutingManager
{
    static auto& routings() { return getGameState().routings; }
    static auto& pathReservedRoutings() { return getGameState().pathReservedRoutings; }
    static std::array<std::deque<uint16_t>, Limits::kMaxVehicles> _continuations;

    static uint64_t getRoutingMask(const RoutingHandle handle)
    {
        return uint64_t{ 1 } << handle.getIndex();
    }

    static bool isValidTrackRouting(const uint16_t routing)
    {
        if (routing == kAllocatedButFreeRouting || routing == kRoutingNull)
        {
            return false;
        }
        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = routing & World::Track::AdditionalTaDFlags::basicTaDMask;
        return tad.id() < World::TrackData::kTrackPieceCount;
    }

    static const VehicleHead* findVehicleHead(const GameState& gameState, const uint16_t vehicleRef)
    {
        for (const auto& entity : gameState.entities)
        {
            const auto* vehicle = entity.asBase<VehicleBase>();
            if (vehicle != nullptr && vehicle->isVehicleHead())
            {
                const auto* head = vehicle->asVehicleHead();
                if (head->routingHandle.getVehicleRef() == vehicleRef)
                {
                    return head;
                }
            }
        }
        return nullptr;
    }

    static bool validateContinuation(const State& state, const GameState& gameState, const uint16_t vehicleRef)
    {
        const auto& continuation = state.continuations[vehicleRef];
        if (continuation.empty())
        {
            return true;
        }
        const auto* head = findVehicleHead(gameState, vehicleRef);
        if (head == nullptr || head->mode != TransportMode::rail || !head->isPlaced()
            || head->status == Status::crashed || head->status == Status::stuck)
        {
            return false;
        }

        const auto& vehicleRoutings = gameState.routings[vehicleRef];
        if (!isValidTrackRouting(vehicleRoutings[head->routingHandle.getIndex()]))
        {
            return false;
        }
        auto index = head->routingHandle.getIndex();
        uint64_t futureMask = 0;
        bool foundFreeSlot = false;
        for (size_t i = 1; i < Limits::kMaxRoutingsPerVehicle; ++i)
        {
            index = (index + 1) & (Limits::kMaxRoutingsPerVehicle - 1);
            const auto routing = vehicleRoutings[index];
            if (routing == kAllocatedButFreeRouting)
            {
                foundFreeSlot = true;
                break;
            }
            if (!isValidTrackRouting(routing))
            {
                return false;
            }
            futureMask |= uint64_t{ 1 } << index;
        }
        if (!foundFreeSlot)
        {
            return false;
        }
        const auto reservationMask = state.pathReservedRoutings[vehicleRef];
        const auto requiredMask = futureMask != 0 ? futureMask : uint64_t{ 1 } << head->routingHandle.getIndex();
        if ((reservationMask & requiredMask) != requiredMask)
        {
            return false;
        }

        for (const auto routing : continuation)
        {
            if (!isValidTrackRouting(routing))
            {
                return false;
            }
        }
        return true;
    }

    static std::optional<uint16_t> findFreeRoutingVehicleRef()
    {
        const auto& routingArr = routings();
        const auto res = std::find_if(std::begin(routingArr), std::end(routingArr), [](const auto& route) { return route[0] == kRoutingNull; });
        if (res == std::end(routingArr))
        {
            return std::nullopt;
        }
        return std::distance(std::begin(routingArr), res);
    }

    void resetRoutings(const RoutingHandle handle)
    {
        auto& vehRoutingArr = routings()[handle.getVehicleRef()];
        std::fill(std::begin(vehRoutingArr), std::end(vehRoutingArr), kAllocatedButFreeRouting);
        PathSignals::markVehicleClaimsDirty(handle.getVehicleRef());
        clearPathReservations(handle);
    }

    bool isEmptyRoutingSlotAvailable()
    {
        return findFreeRoutingVehicleRef().has_value();
    }

    // 0x004B1E00
    std::optional<RoutingHandle> getAndAllocateFreeRoutingHandle()
    {
        auto vehicleRef = findFreeRoutingVehicleRef();
        if (vehicleRef.has_value())
        {
            auto& vehRoutingArr = routings()[*vehicleRef];
            std::fill(std::begin(vehRoutingArr), std::end(vehRoutingArr), kAllocatedButFreeRouting);
            pathReservedRoutings()[*vehicleRef] = 0;
            _continuations[*vehicleRef].clear();
            return { RoutingHandle(*vehicleRef, 0) };
        }
        return std::nullopt;
    }

    uint16_t getRouting(const RoutingHandle handle)
    {
        return routings()[handle.getVehicleRef()][handle.getIndex()];
    }

    void setRouting(const RoutingHandle handle, uint16_t routing)
    {
        auto& currentRouting = routings()[handle.getVehicleRef()][handle.getIndex()];
        auto claimsChanged = currentRouting != routing;
        currentRouting = routing;
        if (routing == kAllocatedButFreeRouting || routing == kRoutingNull)
        {
            auto& reservations = pathReservedRoutings()[handle.getVehicleRef()];
            const auto routingMask = getRoutingMask(handle);
            claimsChanged |= (reservations & routingMask) != 0;
            reservations &= ~routingMask;
        }
        if (claimsChanged)
        {
            PathSignals::markVehicleClaimsDirty(handle.getVehicleRef());
        }
    }

    void freeRouting(const RoutingHandle handle)
    {
        setRouting(handle, kAllocatedButFreeRouting);
    }

    bool materializeReservedContinuation(const VehicleHead& head)
    {
        const auto headHandle = head.routingHandle;
        auto& continuation = _continuations[headHandle.getVehicleRef()];
        if (continuation.empty())
        {
            return true;
        }
        if (head.mode != TransportMode::rail || !head.isPlaced()
            || head.status == Status::crashed || head.status == Status::stuck)
        {
            return false;
        }

        if (!isValidTrackRouting(getRouting(headHandle)))
        {
            return false;
        }
        auto handle = headHandle;
        for (size_t i = 1; i < Limits::kMaxRoutingsPerVehicle; ++i)
        {
            handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
            const auto routing = getRouting(handle);
            if (routing == kAllocatedButFreeRouting)
            {
                break;
            }
            if (!isValidTrackRouting(routing))
            {
                return false;
            }
        }

        size_t freeSlots = 0;
        auto freeHandle = handle;
        for (size_t i = 0; i < Limits::kMaxRoutingsPerVehicle; ++i)
        {
            if (getRouting(handle) != kAllocatedButFreeRouting)
            {
                break;
            }
            ++freeSlots;
            handle.setIndex((handle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
        }

        const auto numToMaterialize = std::min(freeSlots > kRequiredFreeRoutingSlots ? freeSlots - kRequiredFreeRoutingSlots : 0, continuation.size());
        for (size_t i = 0; i < numToMaterialize; ++i)
        {
            if (!isValidTrackRouting(continuation[i]))
            {
                return false;
            }
        }
        for (size_t i = 0; i < numToMaterialize; ++i)
        {
            setRouting(freeHandle, continuation.front());
            markPathReserved(freeHandle);
            continuation.pop_front();
            freeHandle.setIndex((freeHandle.getIndex() + 1) & (Limits::kMaxRoutingsPerVehicle - 1));
        }
        return true;
    }

    void freeTailRoutingAndRefill(const RoutingHandle oldTailHandle, const VehicleHead& head)
    {
        freeRouting(oldTailHandle);
        materializeReservedContinuation(head);
    }

    void markPathReserved(const RoutingHandle handle)
    {
        auto& reservations = pathReservedRoutings()[handle.getVehicleRef()];
        const auto previous = reservations;
        reservations |= getRoutingMask(handle);
        if (reservations != previous)
        {
            PathSignals::markVehicleClaimsDirty(handle.getVehicleRef());
        }
    }

    bool isPathReserved(const RoutingHandle handle)
    {
        return (pathReservedRoutings()[handle.getVehicleRef()] & getRoutingMask(handle)) != 0;
    }

    bool hasPathReservations()
    {
        return std::ranges::any_of(pathReservedRoutings(), [](const auto mask) { return mask != 0; })
            || std::ranges::any_of(_continuations, [](const auto& continuation) { return !continuation.empty(); });
    }

    bool hasPathReservations(const RoutingHandle handle)
    {
        return pathReservedRoutings()[handle.getVehicleRef()] != 0 || !_continuations[handle.getVehicleRef()].empty();
    }

    bool hasPathReservations(const State& state)
    {
        return std::ranges::any_of(state.pathReservedRoutings, [](const auto mask) { return mask != 0; })
            || std::ranges::any_of(state.continuations, [](const auto& continuation) { return !continuation.empty(); });
    }

    void clearPathReservations(const RoutingHandle handle)
    {
        auto& reservations = pathReservedRoutings()[handle.getVehicleRef()];
        if (reservations != 0)
        {
            reservations = 0;
            PathSignals::markVehicleClaimsDirty(handle.getVehicleRef());
        }
        clearReservedContinuation(handle);
    }

    const std::deque<uint16_t>& getReservedContinuation(const RoutingHandle handle)
    {
        return _continuations[handle.getVehicleRef()];
    }

    void setReservedContinuation(const RoutingHandle handle, std::vector<uint16_t> entries)
    {
        auto& continuation = _continuations[handle.getVehicleRef()];
        continuation.assign(entries.begin(), entries.end());
        PathSignals::markVehicleClaimsDirty(handle.getVehicleRef());
    }

    void clearReservedContinuation(const RoutingHandle handle)
    {
        auto& continuation = _continuations[handle.getVehicleRef()];
        if (!continuation.empty())
        {
            continuation.clear();
            PathSignals::markVehicleClaimsDirty(handle.getVehicleRef());
        }
    }

    // 0x004B1E77
    void freeRoutingHandle(const RoutingHandle handle)
    {
        auto& vehRoutingArr = routings()[handle.getVehicleRef()];
        std::fill(std::begin(vehRoutingArr), std::end(vehRoutingArr), kRoutingNull);
        PathSignals::markVehicleClaimsDirty(handle.getVehicleRef());
        clearPathReservations(handle);
    }

    // 0x004A8810
    void resetRoutingTable()
    {
        std::fill_n(&routings()[0][0], Limits::kMaxVehicles * Limits::kMaxRoutingsPerVehicle, kRoutingNull);
        resetPathReservationState();
    }

    void resetPathReservationState()
    {
        std::fill(std::begin(pathReservedRoutings()), std::end(pathReservedRoutings()), 0);
        for (auto& continuation : _continuations)
        {
            continuation.clear();
        }
    }

    State captureState()
    {
        State state;
        std::ranges::copy(pathReservedRoutings(), state.pathReservedRoutings.begin());
        for (size_t vehicleRef = 0; vehicleRef < _continuations.size(); ++vehicleRef)
        {
            state.continuations[vehicleRef].assign(_continuations[vehicleRef].begin(), _continuations[vehicleRef].end());
        }
        return state;
    }

    bool validateState(const State& state)
    {
        return validateState(state, getGameState());
    }

    bool validateState(const State& state, const GameState& gameState)
    {
        for (size_t vehicleRef = 0; vehicleRef < state.pathReservedRoutings.size(); ++vehicleRef)
        {
            auto reservationMask = state.pathReservedRoutings[vehicleRef];
            while (reservationMask != 0)
            {
                const auto routing = gameState.routings[vehicleRef][std::countr_zero(reservationMask)];
                if (!isValidTrackRouting(routing))
                {
                    return false;
                }
                reservationMask &= reservationMask - 1;
            }
            const auto& continuation = state.continuations[vehicleRef];
            if (continuation.size() > kMaxContinuationEntriesPerVehicle
                || (!continuation.empty() && (state.pathReservedRoutings[vehicleRef] == 0 || gameState.routings[vehicleRef][0] == kRoutingNull))
                || std::ranges::any_of(continuation, [](const auto routing) { return !isValidTrackRouting(routing); }))
            {
                return false;
            }
            if (!validateContinuation(state, gameState, static_cast<uint16_t>(vehicleRef)))
            {
                return false;
            }
        }
        return true;
    }

    bool restoreState(const State& state)
    {
        if (!validateState(state))
        {
            return false;
        }
        decltype(_continuations) restoredContinuations;
        for (size_t vehicleRef = 0; vehicleRef < state.continuations.size(); ++vehicleRef)
        {
            restoredContinuations[vehicleRef].assign(state.continuations[vehicleRef].begin(), state.continuations[vehicleRef].end());
        }
        std::ranges::copy(state.pathReservedRoutings, std::begin(pathReservedRoutings()));
        _continuations.swap(restoredContinuations);
        return true;
    }

    RingView::Iterator::Iterator(const RoutingHandle& begin, bool isEnd, Direction direction)
        : _current(begin)
        , _isEnd(isEnd)
        , _direction(direction)
    {
        if (routings()[_current.getVehicleRef()][_current.getIndex()] == kAllocatedButFreeRouting)
        {
            _hasLooped = true;
        }
    }

    RingView::Iterator& RingView::Iterator::operator++()
    {
        if (_direction == Direction::reverse)
        {
            return --*this;
        }
        _current.setIndex((_current.getIndex() + 1) & 0x3F);

        if (_current.getIndex() == 0)
        {
            _hasLooped = true;
        }
        return *this;
    }

    RingView::Iterator& RingView::Iterator::operator--()
    {
        _current.setIndex((_current.getIndex() - 1) & 0x3F);

        if (_current.getIndex() == 0x3F)
        {
            _hasLooped = true;
        }
        return *this;
    }

    bool RingView::Iterator::operator==(const RingView::Iterator& other) const
    {
        if ((_hasLooped || other._hasLooped) && _current == other._current)
        {
            return true;
        }
        // If this is an end iterator then its value is implied to be kAllocatedButFreeRouting
        if (_isEnd)
        {
            return routings()[other._current.getVehicleRef()][other._current.getIndex()] == kAllocatedButFreeRouting;
        }
        if (other._isEnd)
        {
            return routings()[_current.getVehicleRef()][_current.getIndex()] == kAllocatedButFreeRouting;
        }
        return false;
    }
}
