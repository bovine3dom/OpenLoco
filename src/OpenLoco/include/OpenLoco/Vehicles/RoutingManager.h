#pragma once

#include "Routing.h"

#include <array>
#include <deque>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace OpenLoco
{
    struct GameState;
}

namespace OpenLoco::Vehicles
{
    struct VehicleHead;
}

namespace OpenLoco::Vehicles::RoutingManager
{
    constexpr uint16_t kAllocatedButFreeRouting = 0xFFFEU; // Indicates that this array entry is allocated to a vehicle but no routing has been set.
    constexpr uint16_t kRoutingNull = 0xFFFFU;             // Indicates that this array entry is unallocated to any vehicle.
    constexpr size_t kRequiredFreeRoutingSlots = 3;
    constexpr size_t kMaxContinuationEntriesPerVehicle = 4096;
    constexpr size_t kMaxContinuationEntries = Limits::kMaxVehicles * kMaxContinuationEntriesPerVehicle;
    constexpr size_t kMaxSaveDataSize = sizeof(uint64_t) * Limits::kMaxVehicles + sizeof(uint16_t) + sizeof(uint16_t) * 2 * Limits::kMaxVehicles + sizeof(uint16_t) * kMaxContinuationEntries;

    struct State
    {
        std::array<uint64_t, Limits::kMaxVehicles> pathReservedRoutings{};
        std::array<std::vector<uint16_t>, Limits::kMaxVehicles> continuations{};

        bool operator==(const State&) const = default;
    };

    std::optional<RoutingHandle> getAndAllocateFreeRoutingHandle();
    void freeRoutingHandle(const RoutingHandle handle);
    // Returns a routing. Each routing represents a track/road piece that a train is on or has reserved
    // See OpenLoco::World::Track::AdditionalTadFlags for bits of the routing
    uint16_t getRouting(const RoutingHandle handle);
    void setRouting(const RoutingHandle handle, uint16_t routing);
    void freeRouting(const RoutingHandle handle);
    bool materializeReservedContinuation(const VehicleHead& head);
    void freeTailRoutingAndRefill(RoutingHandle oldTailHandle, const VehicleHead& head);
    void markPathReserved(const RoutingHandle handle);
    bool isPathReserved(const RoutingHandle handle);
    bool hasPathReservations();
    bool hasPathReservations(const RoutingHandle handle);
    bool hasPathReservations(const State& state);
    void clearPathReservations(const RoutingHandle handle);
    const std::deque<uint16_t>& getReservedContinuation(RoutingHandle handle);
    void setReservedContinuation(RoutingHandle handle, std::vector<uint16_t> entries);
    void clearReservedContinuation(RoutingHandle handle);
    // Equivalent of calling freeRouting on all routings for a single vehicle
    void resetRoutings(const RoutingHandle handle);
    bool isEmptyRoutingSlotAvailable();
    void resetRoutingTable();
    void resetPathReservationState();
    State captureState();
    bool validateState(const State& state);
    bool validateState(const State& state, const GameState& gameState);
    bool restoreState(const State& state);

    struct RingView
    {
    private:
        struct Iterator
        {
            enum class Direction : bool
            {
                forward,
                reverse,
            };

        private:
            RoutingHandle _current;
            bool _hasLooped = false;
            bool _isEnd = false;
            Direction _direction = Direction::forward;

        public:
            Iterator(const RoutingHandle& begin, bool isEnd, Direction direction);

            Iterator& operator++();
            Iterator operator++(int)
            {
                Iterator res = *this;
                ++(*this);
                return res;
            }

            Iterator& operator--();
            Iterator operator--(int)
            {
                Iterator res = *this;
                --(*this);
                return res;
            }

            bool operator==(const Iterator& other) const;

            RoutingHandle operator*() const
            {
                return _current;
            }

            RoutingHandle& operator->()
            {
                return _current;
            }

            const RoutingHandle& operator->() const
            {
                return _current;
            }

            // iterator traits
            using difference_type = std::ptrdiff_t;
            using value_type = RoutingHandle;
            using pointer = const RoutingHandle*;
            using reference = const RoutingHandle&;
            using iterator_category = std::bidirectional_iterator_tag;
        };

        RoutingHandle _begin;

    public:
        // currentOrderOffset is relative to beginTableOffset and is where the ring will begin and end
        RingView(const RoutingHandle begin)
            : _begin(begin)
        {
        }

        RingView::Iterator begin() const { return Iterator(_begin, false, Iterator::Direction::forward); }
        RingView::Iterator end() const { return Iterator(_begin, true, Iterator::Direction::forward); }
        auto rbegin() const { return Iterator(_begin, false, Iterator::Direction::reverse); }
        auto rend() const { return Iterator(_begin, true, Iterator::Direction::reverse); }
    };
}
