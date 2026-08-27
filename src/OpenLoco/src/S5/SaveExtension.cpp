// SPDX-License-Identifier: MIT
#include <OpenLoco/S5/SaveExtension.h>

#include "S5/Limits.h"
#include "S5/S5Station.h"
#include <OpenLoco/CargoDist/Save.h>
#include <OpenLoco/GameCommands/Vehicles/VehiclePlace.h>
#include <OpenLoco/World/Station.h>
#include <algorithm>
#include <array>
#include <bit>
#include <cstring>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

namespace OpenLoco::S5::SaveExtension
{
    bool VehicleObjectSlot::operator==(const VehicleObjectSlot& rhs) const
    {
        return slot == rhs.slot && std::memcmp(&header, &rhs.header, sizeof(header)) == 0;
    }

    bool VehicleObjectState::operator==(const VehicleObjectState& rhs) const
    {
        if (objects != rhs.objects)
        {
            return false;
        }
        for (size_t company = 0; company < companyUnlocks.size(); ++company)
        {
            if (companyUnlocks[company].data() != rhs.companyUnlocks[company].data())
            {
                return false;
            }
        }
        return true;
    }

    namespace
    {
        constexpr std::array<std::byte, 8> kMagic = {
            std::byte{ 'O' },
            std::byte{ 'L' },
            std::byte{ 'E' },
            std::byte{ 'X' },
            std::byte{ 'T' },
            std::byte{ 'E' },
            std::byte{ 'N' },
            std::byte{ 'D' },
        };
        constexpr std::array<std::byte, 8> kLegacyCargoDistMagic = {
            std::byte{ 'O' },
            std::byte{ 'L' },
            std::byte{ 'C' },
            std::byte{ 'D' },
            std::byte{ 'I' },
            std::byte{ 'S' },
            std::byte{ 'T' },
            std::byte{ 0 },
        };
        constexpr std::array<std::byte, 4> kCargoDistTag = {
            std::byte{ 'C' },
            std::byte{ 'D' },
            std::byte{ 'S' },
            std::byte{ 'T' },
        };
        constexpr std::array<std::byte, 4> kSharedOrdersTag = {
            std::byte{ 'S' },
            std::byte{ 'H' },
            std::byte{ 'O' },
            std::byte{ 'R' },
        };
        constexpr std::array<std::byte, 4> kPathReservationsTag = {
            std::byte{ 'P' },
            std::byte{ 'R' },
            std::byte{ 'E' },
            std::byte{ 'S' },
        };
        constexpr std::array<std::byte, 4> kVehicleAutoRenewalTag = {
            std::byte{ 'V' },
            std::byte{ 'R' },
            std::byte{ 'E' },
            std::byte{ 'N' },
        };
        constexpr std::array<std::byte, 4> kRailTrafficTag = {
            std::byte{ 'R' },
            std::byte{ 'T' },
            std::byte{ 'F' },
            std::byte{ 'C' },
        };
        constexpr std::array<std::byte, 4> kVehicleReplacementTag = {
            std::byte{ 'V' },
            std::byte{ 'R' },
            std::byte{ 'P' },
            std::byte{ 'L' },
        };
        constexpr std::array<std::byte, 4> kStationTileOverflowTag = {
            std::byte{ 'S' },
            std::byte{ 'T' },
            std::byte{ 'N' },
            std::byte{ 'S' },
        };
        constexpr std::array<std::byte, 4> kGameRulesTag = {
            std::byte{ 'R' },
            std::byte{ 'U' },
            std::byte{ 'L' },
            std::byte{ 'E' },
        };
        constexpr std::array<std::byte, 4> kVehicleObjectsTag = {
            std::byte{ 'V' },
            std::byte{ 'O' },
            std::byte{ 'B' },
            std::byte{ 'J' },
        };
        constexpr std::array<std::byte, 4> kTimetableTag = {
            std::byte{ 'T' },
            std::byte{ 'T' },
            std::byte{ 'B' },
            std::byte{ 'L' },
        };
        constexpr uint16_t kVersion = 1;
        constexpr uint16_t kHeaderSize = 16;
        constexpr uint16_t kSectionVersion = 1;
        // Version 1 reservations may contain routes that bypass their active waypoint.
        constexpr uint16_t kPathReservationsSectionVersion = 2;
        constexpr uint16_t kPathReservationContinuationsSectionVersion = 3;
        // Version 2 additionally stores deferred replacement vehicle placements.
        constexpr uint16_t kVehicleReplacementPendingPlacementsVersion = 2;
        // Version 3 additionally stores whether a deferred placement should start.
        constexpr uint16_t kVehicleReplacementSectionVersion = 3;
        constexpr uint16_t kSectionRequired = 1U << 0;
        constexpr uint16_t kKnownSectionFlags = kSectionRequired;
        constexpr size_t kSectionHeaderSize = 12;
        constexpr size_t kVehicleUnlockWordBits = 64;
        constexpr size_t kVehicleUnlockWordCount = (kExtendedVehicleObjectCount + kVehicleUnlockWordBits - 1) / kVehicleUnlockWordBits;

        static_assert(kVehicleUnlockWordCount == 13);

        void require(bool condition, const char* message)
        {
            if (!condition)
            {
                throw std::runtime_error(message);
            }
        }

        class Writer
        {
        public:
            template<typename T>
            void write(T value)
            {
                static_assert(std::is_unsigned_v<T>);
                require(sizeof(T) <= kMaxDataSize - _data.size(), "Save extension is too large");
                for (size_t i = 0; i < sizeof(T); ++i)
                {
                    _data.push_back(static_cast<std::byte>((value >> (i * 8)) & 0xFF));
                }
            }

            void writeBytes(std::span<const std::byte> bytes)
            {
                require(bytes.size() <= kMaxDataSize - _data.size(), "Save extension is too large");
                _data.insert(_data.end(), bytes.begin(), bytes.end());
            }

            void writeSigned(const int64_t value)
            {
                write(std::bit_cast<uint64_t>(value));
            }

            size_t size() const { return _data.size(); }
            const std::vector<std::byte>& data() const { return _data; }
            std::vector<std::byte> take() { return std::move(_data); }

        private:
            std::vector<std::byte> _data;
        };

        class Reader
        {
        public:
            explicit Reader(std::span<const std::byte> data)
                : _data(data)
            {
            }

            template<typename T>
            T read()
            {
                static_assert(std::is_unsigned_v<T>);
                requireRemaining(sizeof(T));
                T value{};
                for (size_t i = 0; i < sizeof(T); ++i)
                {
                    value |= static_cast<T>(std::to_integer<uint8_t>(_data[_position++])) << (i * 8);
                }
                return value;
            }

            std::span<const std::byte> readBytes(size_t size)
            {
                requireRemaining(size);
                const auto bytes = _data.subspan(_position, size);
                _position += size;
                return bytes;
            }

            int64_t readSigned()
            {
                return std::bit_cast<int64_t>(read<uint64_t>());
            }

            size_t remaining() const { return _data.size() - _position; }
            bool empty() const { return remaining() == 0; }

        private:
            void requireRemaining(size_t size) const
            {
                require(size <= remaining(), "Truncated save extension");
            }

            std::span<const std::byte> _data;
            size_t _position{};
        };

        void appendSection(Writer& output, std::span<const std::byte, 4> tag, std::span<const std::byte> payload, uint16_t flags = 0, uint16_t version = kSectionVersion)
        {
            require(payload.size() <= std::numeric_limits<uint32_t>::max(), "Save extension section is too large");
            require(kSectionHeaderSize <= kMaxDataSize - output.size() && payload.size() <= kMaxDataSize - output.size() - kSectionHeaderSize, "Save extension is too large");
            output.writeBytes(tag);
            output.write(version);
            output.write(flags);
            output.write(static_cast<uint32_t>(payload.size()));
            output.writeBytes(payload);
        }

        std::vector<std::byte> encodeSharedOrders(const Vehicles::SharedOrderManager::State& state)
        {
            std::vector<std::vector<uint16_t>> groups;
            groups.reserve(state.groups.size());
            for (const auto& group : state.groups)
            {
                require(group.members.size() >= 2 && group.members.size() <= Limits::kMaxVehicles, "Invalid shared order group size");
                auto& members = groups.emplace_back();
                members.reserve(group.members.size());
                for (const auto id : group.members)
                {
                    const auto value = static_cast<uint16_t>(id);
                    require(value < Limits::kMaxEntities, "Invalid shared order entity ID");
                    members.push_back(value);
                }
                std::ranges::sort(members);
                require(std::ranges::adjacent_find(members) == members.end(), "Duplicate shared order entity ID");
            }
            std::ranges::sort(groups);
            require(groups.size() <= Limits::kMaxVehicles / 2, "Too many shared order groups");

            std::array<bool, Limits::kMaxEntities> seen{};
            size_t totalMembers = 0;
            Writer payload;
            payload.write(static_cast<uint32_t>(groups.size()));
            for (const auto& members : groups)
            {
                require(members.size() <= Limits::kMaxVehicles - totalMembers, "Too many shared order members");
                totalMembers += members.size();
                payload.write(static_cast<uint32_t>(members.size()));
                for (const auto id : members)
                {
                    require(!seen[id], "Entity belongs to multiple shared order groups");
                    seen[id] = true;
                    payload.write(id);
                }
            }
            return payload.take();
        }

        Vehicles::SharedOrderManager::State decodeSharedOrders(std::span<const std::byte> data)
        {
            Reader input(data);
            const auto groupCount = input.read<uint32_t>();
            require(groupCount <= Limits::kMaxVehicles / 2, "Too many shared order groups");

            Vehicles::SharedOrderManager::State state;
            state.groups.reserve(groupCount);
            std::array<bool, Limits::kMaxEntities> seen{};
            size_t totalMembers = 0;
            uint16_t previousLeader{};
            bool hasPreviousLeader = false;
            for (uint32_t i = 0; i < groupCount; ++i)
            {
                const auto memberCount = input.read<uint32_t>();
                require(memberCount >= 2 && memberCount <= Limits::kMaxVehicles - totalMembers, "Invalid shared order group size");
                require(memberCount <= input.remaining() / sizeof(uint16_t), "Truncated shared order group");
                totalMembers += memberCount;

                auto& members = state.groups.emplace_back().members;
                members.reserve(memberCount);
                uint16_t previousMember{};
                for (uint32_t j = 0; j < memberCount; ++j)
                {
                    const auto id = input.read<uint16_t>();
                    require(id < Limits::kMaxEntities, "Invalid shared order entity ID");
                    require(j == 0 || previousMember < id, "Non-canonical shared order group");
                    require(!seen[id], "Entity belongs to multiple shared order groups");
                    seen[id] = true;
                    members.push_back(static_cast<EntityId>(id));
                    previousMember = id;
                }
                const auto leader = static_cast<uint16_t>(members.front());
                require(!hasPreviousLeader || previousLeader < leader, "Non-canonical shared order group order");
                previousLeader = leader;
                hasPreviousLeader = true;
            }
            require(input.empty(), "Trailing shared order data");
            return state;
        }

        std::vector<std::byte> encodePathReservations(const Vehicles::RoutingManager::State& state)
        {
            Writer payload;
            for (const auto mask : state.pathReservedRoutings)
            {
                payload.write(mask);
            }
            const auto continuationCount = static_cast<uint16_t>(std::ranges::count_if(state.continuations, [](const auto& continuation) { return !continuation.empty(); }));
            if (continuationCount != 0)
            {
                payload.write(continuationCount);
                for (size_t vehicleRef = 0; vehicleRef < state.continuations.size(); ++vehicleRef)
                {
                    const auto& continuation = state.continuations[vehicleRef];
                    if (continuation.empty())
                    {
                        continue;
                    }
                    require(state.pathReservedRoutings[vehicleRef] != 0, "Path continuation has no materialized reservation");
                    require(continuation.size() <= Vehicles::RoutingManager::kMaxContinuationEntriesPerVehicle, "Too many path continuation entries");
                    payload.write(static_cast<uint16_t>(vehicleRef));
                    payload.write(static_cast<uint16_t>(continuation.size()));
                    for (const auto routing : continuation)
                    {
                        require(routing != Vehicles::RoutingManager::kAllocatedButFreeRouting && routing != Vehicles::RoutingManager::kRoutingNull, "Invalid path continuation routing");
                        payload.write(routing);
                    }
                }
            }
            return payload.take();
        }

        Vehicles::RoutingManager::State decodePathReservations(std::span<const std::byte> data, const uint16_t version)
        {
            constexpr auto kMaskDataSize = Limits::kMaxVehicles * sizeof(uint64_t);
            require(data.size() >= kMaskDataSize, "Invalid path reservation data size");
            Reader input(data);
            Vehicles::RoutingManager::State state;
            for (auto& mask : state.pathReservedRoutings)
            {
                mask = input.read<uint64_t>();
            }
            if (version != kPathReservationContinuationsSectionVersion)
            {
                require(input.empty(), "Invalid path reservation data size");
                return state;
            }

            const auto continuationCount = input.read<uint16_t>();
            require(continuationCount != 0 && continuationCount <= Limits::kMaxVehicles, "Invalid path continuation count");
            uint16_t previousVehicleRef = 0;
            for (uint16_t i = 0; i < continuationCount; ++i)
            {
                const auto vehicleRef = input.read<uint16_t>();
                const auto entryCount = input.read<uint16_t>();
                require(vehicleRef < Limits::kMaxVehicles && (i == 0 || previousVehicleRef < vehicleRef), "Non-canonical path continuation order");
                require(entryCount != 0 && entryCount <= Vehicles::RoutingManager::kMaxContinuationEntriesPerVehicle, "Invalid path continuation size");
                require(entryCount <= input.remaining() / sizeof(uint16_t), "Truncated path continuation");
                require(state.pathReservedRoutings[vehicleRef] != 0, "Path continuation has no materialized reservation");
                auto& continuation = state.continuations[vehicleRef];
                continuation.reserve(entryCount);
                for (uint16_t j = 0; j < entryCount; ++j)
                {
                    const auto routing = input.read<uint16_t>();
                    require(routing != Vehicles::RoutingManager::kAllocatedButFreeRouting && routing != Vehicles::RoutingManager::kRoutingNull, "Invalid path continuation routing");
                    continuation.push_back(routing);
                }
                previousVehicleRef = vehicleRef;
            }
            require(input.empty(), "Trailing path reservation data");
            return state;
        }

        std::vector<std::byte> encodeVehicleAutoRenewal(const Vehicles::VehicleAutoRenewal::State& state)
        {
            require(Vehicles::VehicleAutoRenewal::validateState(state), "Invalid vehicle auto-renewal state");
            Writer payload;
            for (const auto& settings : state.companies)
            {
                payload.write(static_cast<uint8_t>(settings.enabled));
                payload.write(settings.reliabilityThreshold);
            }
            return payload.take();
        }

        Vehicles::VehicleAutoRenewal::State decodeVehicleAutoRenewal(std::span<const std::byte> data)
        {
            constexpr auto kEntrySize = sizeof(uint8_t) * 2;
            require(data.size() == Limits::kMaxCompanies * kEntrySize, "Invalid vehicle auto-renewal data size");

            Reader input(data);
            Vehicles::VehicleAutoRenewal::State state;
            for (auto& settings : state.companies)
            {
                const auto enabled = input.read<uint8_t>();
                require(enabled <= 1, "Invalid vehicle auto-renewal enabled value");
                settings.enabled = enabled != 0;
                settings.reliabilityThreshold = input.read<uint8_t>();
            }
            require(Vehicles::VehicleAutoRenewal::validateState(state), "Invalid vehicle auto-renewal state");
            return state;
        }

        std::vector<std::byte> encodeVehicleReplacement(const Vehicles::VehicleReplacement::State& state)
        {
            require(Vehicles::VehicleReplacement::validateState(state), "Invalid vehicle replacement state");
            require(state.requests.size() <= std::numeric_limits<uint16_t>::max() && state.pendingPlacements.size() <= std::numeric_limits<uint16_t>::max(), "Too many vehicle replacements");
            Writer payload;
            payload.write(static_cast<uint16_t>(state.requests.size()));
            for (const auto& request : state.requests)
            {
                payload.write(enumValue(request.target));
                payload.write(enumValue(request.source));
            }
            payload.write(static_cast<uint16_t>(state.pendingPlacements.size()));
            for (const auto& placement : state.pendingPlacements)
            {
                payload.write(static_cast<uint16_t>(placement.args.pos.x));
                payload.write(static_cast<uint16_t>(placement.args.pos.y));
                payload.write(static_cast<uint16_t>(placement.args.pos.z));
                payload.write(placement.args.trackAndDirection);
                payload.write(placement.args.trackProgress);
                payload.write(enumValue(placement.args.head));
                payload.write(static_cast<uint8_t>(placement.start));
            }
            return payload.take();
        }

        Vehicles::VehicleReplacement::State decodeVehicleReplacement(const std::span<const std::byte> data, const uint16_t version)
        {
            require(version == kSectionVersion || version == kVehicleReplacementPendingPlacementsVersion || version == kVehicleReplacementSectionVersion, "Unsupported vehicle replacement section version");
            Reader input(data);
            Vehicles::VehicleReplacement::State state;
            const auto count = input.read<uint16_t>();
            state.requests.reserve(count);
            for (uint16_t i = 0; i < count; ++i)
            {
                state.requests.push_back({ EntityId(input.read<uint16_t>()), EntityId(input.read<uint16_t>()) });
            }
            if (version == kVehicleReplacementPendingPlacementsVersion || version == kVehicleReplacementSectionVersion)
            {
                const auto pendingCount = input.read<uint16_t>();
                state.pendingPlacements.reserve(pendingCount);
                for (uint16_t i = 0; i < pendingCount; ++i)
                {
                    Vehicles::VehicleReplacement::State::PendingPlacement placement;
                    const auto x = static_cast<int16_t>(input.read<uint16_t>());
                    const auto y = static_cast<int16_t>(input.read<uint16_t>());
                    const auto z = static_cast<int16_t>(input.read<uint16_t>());
                    placement.args.pos = World::Pos3(x, y, z);
                    placement.args.trackAndDirection = input.read<uint16_t>();
                    placement.args.trackProgress = input.read<uint16_t>();
                    placement.args.head = EntityId(input.read<uint16_t>());
                    if (version == kVehicleReplacementSectionVersion)
                    {
                        const auto start = input.read<uint8_t>();
                        require(start <= 1, "Invalid pending vehicle placement start value");
                        placement.start = start != 0;
                    }
                    state.pendingPlacements.push_back(placement);
                }
            }
            require(input.empty() && Vehicles::VehicleReplacement::validateState(state), "Invalid vehicle replacement state");
            return state;
        }

        void writeRailTrafficEdge(Writer& output, const Vehicles::RailTraffic::Edge& edge)
        {
            output.write(static_cast<uint16_t>(edge.x));
            output.write(static_cast<uint16_t>(edge.y));
            output.write(static_cast<uint16_t>(edge.z));
            output.write(edge.tad);
            output.write(edge.trackType);
        }

        Vehicles::RailTraffic::Edge readRailTrafficEdge(Reader& input)
        {
            return {
                static_cast<int16_t>(input.read<uint16_t>()),
                static_cast<int16_t>(input.read<uint16_t>()),
                static_cast<int16_t>(input.read<uint16_t>()),
                input.read<uint16_t>(),
                input.read<uint8_t>(),
            };
        }

        bool railTrafficEdgeLess(const Vehicles::RailTraffic::HistoryEntry& lhs, const Vehicles::RailTraffic::HistoryEntry& rhs)
        {
            const auto& a = lhs.edge;
            const auto& b = rhs.edge;
            return std::tie(a.x, a.y, a.z, a.trackType, a.tad) < std::tie(b.x, b.y, b.z, b.trackType, b.tad);
        }

        std::vector<std::byte> encodeRailTraffic(Vehicles::RailTraffic::State state)
        {
            require(Vehicles::RailTraffic::validateState(state), "Invalid rail traffic state");
            std::ranges::sort(state.history, railTrafficEdgeLess);
            std::ranges::sort(state.active, {}, &Vehicles::RailTraffic::ActiveTraversal::vehicle);

            Writer payload;
            payload.write(static_cast<uint32_t>(state.history.size()));
            for (const auto& entry : state.history)
            {
                writeRailTrafficEdge(payload, entry.edge);
                payload.write(entry.meanTraversalTime);
                payload.write(entry.lastObservedDay);
                payload.write(entry.confidence);
            }
            payload.write(static_cast<uint16_t>(state.active.size()));
            for (const auto& traversal : state.active)
            {
                payload.write(static_cast<uint16_t>(traversal.vehicle));
                payload.write(static_cast<uint16_t>(traversal.head));
                writeRailTrafficEdge(payload, traversal.edge);
                payload.write(traversal.enteredAt);
                payload.write(static_cast<uint8_t>(traversal.completeFromStart));
            }
            return payload.take();
        }

        Vehicles::RailTraffic::State decodeRailTraffic(std::span<const std::byte> data)
        {
            Reader input(data);
            Vehicles::RailTraffic::State state;
            const auto historySize = input.read<uint32_t>();
            require(historySize <= Vehicles::RailTraffic::kMaxHistoryEntries, "Too many rail traffic history entries");
            state.history.reserve(historySize);
            for (uint32_t i = 0; i < historySize; ++i)
            {
                Vehicles::RailTraffic::HistoryEntry entry;
                entry.edge = readRailTrafficEdge(input);
                entry.meanTraversalTime = input.read<uint64_t>();
                entry.lastObservedDay = input.read<uint32_t>();
                entry.confidence = input.read<uint8_t>();
                state.history.push_back(entry);
            }
            const auto activeSize = input.read<uint16_t>();
            require(activeSize <= Limits::kMaxVehicles, "Too many active rail traversals");
            state.active.reserve(activeSize);
            for (uint16_t i = 0; i < activeSize; ++i)
            {
                Vehicles::RailTraffic::ActiveTraversal traversal;
                traversal.vehicle = static_cast<EntityId>(input.read<uint16_t>());
                traversal.head = static_cast<EntityId>(input.read<uint16_t>());
                traversal.edge = readRailTrafficEdge(input);
                traversal.enteredAt = input.read<uint64_t>();
                const auto complete = input.read<uint8_t>();
                require(complete <= 1, "Invalid active rail traversal flag");
                traversal.completeFromStart = complete != 0;
                state.active.push_back(traversal);
            }
            require(input.empty(), "Trailing rail traffic data");
            require(Vehicles::RailTraffic::validateState(state), "Invalid rail traffic state");
            require(std::ranges::is_sorted(state.history, railTrafficEdgeLess), "Non-canonical rail traffic history order");
            require(std::ranges::is_sorted(state.active, {}, &Vehicles::RailTraffic::ActiveTraversal::vehicle), "Non-canonical active rail traversal order");
            return state;
        }

        constexpr uint8_t kTimetableEntryHasTravel = 1U << 0;
        constexpr uint8_t kTimetableEntryHasDwell = 1U << 1;
        constexpr uint8_t kTimetableEntryHasDispatch = 1U << 2;
        constexpr uint8_t kKnownTimetableEntryFlags = kTimetableEntryHasTravel | kTimetableEntryHasDwell | kTimetableEntryHasDispatch;
        constexpr uint8_t kDispatchHasLastClaimedMinute = 1U << 0;
        constexpr uint8_t kKnownDispatchFlags = kDispatchHasLastClaimedMinute;
        constexpr uint8_t kRuntimeHasAssignedSlot = 1U << 0;
        constexpr uint8_t kRuntimeTimetableStarted = 1U << 1;
        constexpr uint8_t kRuntimeAtTimedStop = 1U << 2;
        constexpr uint8_t kRuntimeReleased = 1U << 3;
        constexpr uint8_t kRuntimeWaiting = 1U << 4;
        constexpr uint8_t kKnownRuntimeFlags = kRuntimeHasAssignedSlot | kRuntimeTimetableStarted | kRuntimeAtTimedStop | kRuntimeReleased | kRuntimeWaiting;

        void canonicaliseTimetable(Vehicles::TimetableManager::State& state)
        {
            std::ranges::sort(state.services, {}, &Vehicles::TimetableManager::Service::id);
            for (auto& service : state.services)
            {
                std::ranges::sort(service.entries, {}, &Vehicles::TimetableManager::TimetableEntry::orderIndex);
                for (auto& entry : service.entries)
                {
                    if (entry.dispatch.has_value())
                    {
                        std::ranges::sort(entry.dispatch->slots);
                    }
                }
            }
            const auto vehicleId = [](const auto& value) { return enumValue(value.vehicle); };
            std::ranges::sort(state.assignments, {}, vehicleId);
            std::ranges::sort(state.vehicles, {}, vehicleId);
        }

        std::vector<std::byte> encodeTimetable(Vehicles::TimetableManager::State state)
        {
            canonicaliseTimetable(state);
            require(Vehicles::TimetableManager::validateState(state), "Invalid timetable state");

            Writer payload;
            payload.write(state.ticksPerMinute);
            payload.write(state.clockTicks);
            payload.write(state.nextServiceId);
            payload.write(state.nextEntryId);
            payload.write(static_cast<uint32_t>(state.services.size()));
            for (const auto& service : state.services)
            {
                payload.write(service.id);
                payload.write(service.revision);
                payload.write(static_cast<uint16_t>(service.entries.size()));
                for (const auto& entry : service.entries)
                {
                    payload.write(entry.id);
                    payload.write(entry.orderIndex);
                    payload.write(enumValue(entry.orderType));
                    payload.write(enumValue(entry.station));
                    const auto entryFlags = static_cast<uint8_t>((entry.travelMinutes.has_value() ? kTimetableEntryHasTravel : 0) | (entry.dwellMinutes.has_value() ? kTimetableEntryHasDwell : 0) | (entry.dispatch.has_value() ? kTimetableEntryHasDispatch : 0));
                    payload.write(entryFlags);
                    if (entry.travelMinutes.has_value())
                    {
                        payload.write(*entry.travelMinutes);
                    }
                    if (entry.dwellMinutes.has_value())
                    {
                        payload.write(*entry.dwellMinutes);
                    }
                    if (entry.dispatch.has_value())
                    {
                        const auto& dispatch = *entry.dispatch;
                        payload.write(static_cast<uint8_t>(dispatch.lastClaimedMinute.has_value() ? kDispatchHasLastClaimedMinute : 0));
                        payload.write(dispatch.periodMinutes);
                        payload.write(dispatch.phaseMinutes);
                        payload.write(dispatch.maxDelayMinutes);
                        payload.write(static_cast<uint16_t>(dispatch.slots.size()));
                        if (dispatch.lastClaimedMinute.has_value())
                        {
                            payload.writeSigned(*dispatch.lastClaimedMinute);
                        }
                        for (const auto slot : dispatch.slots)
                        {
                            payload.write(slot);
                        }
                    }
                }
            }

            payload.write(static_cast<uint32_t>(state.assignments.size()));
            for (const auto& assignment : state.assignments)
            {
                payload.write(enumValue(assignment.vehicle));
                payload.write(assignment.service);
            }

            payload.write(static_cast<uint32_t>(state.vehicles.size()));
            for (const auto& runtime : state.vehicles)
            {
                payload.write(enumValue(runtime.vehicle));
                payload.write(runtime.service);
                payload.write(runtime.serviceRevision);
                payload.write(runtime.currentEntry);
                payload.write(runtime.scheduledArrivalTick);
                payload.write(runtime.scheduledDepartureTick);
                const auto runtimeFlags = static_cast<uint8_t>((runtime.assignedSlotMinute.has_value() ? kRuntimeHasAssignedSlot : 0) | (runtime.timetableStarted ? kRuntimeTimetableStarted : 0) | (runtime.atTimedStop ? kRuntimeAtTimedStop : 0) | (runtime.released ? kRuntimeReleased : 0) | (runtime.waiting ? kRuntimeWaiting : 0));
                payload.write(runtimeFlags);
                if (runtime.assignedSlotMinute.has_value())
                {
                    payload.writeSigned(*runtime.assignedSlotMinute);
                }
                payload.writeSigned(runtime.latenessTicks);
            }
            require(payload.size() <= kMaxTimetableDataSize, "Timetable state is too large");
            return payload.take();
        }

        Vehicles::TimetableManager::State decodeTimetable(const std::span<const std::byte> data)
        {
            constexpr size_t kMaxEntries = Limits::kMaxVehicles * Limits::kMaxOrdersPerVehicle;
            constexpr size_t kMinServiceSize = sizeof(uint32_t) * 2 + sizeof(uint16_t);
            constexpr size_t kMinEntrySize = sizeof(uint32_t) + sizeof(uint8_t) * 3 + sizeof(uint16_t);
            constexpr size_t kAssignmentSize = sizeof(uint16_t) + sizeof(uint32_t);
            constexpr size_t kMinRuntimeSize = sizeof(uint16_t) + sizeof(uint32_t) * 3 + sizeof(uint64_t) * 3 + sizeof(uint8_t);

            require(data.size() <= kMaxTimetableDataSize, "Timetable state is too large");
            Reader input(data);
            Vehicles::TimetableManager::State state;
            state.ticksPerMinute = input.read<uint16_t>();
            state.clockTicks = input.read<uint64_t>();
            state.nextServiceId = input.read<uint32_t>();
            state.nextEntryId = input.read<uint32_t>();
            require(state.nextEntryId != Vehicles::TimetableManager::kInvalidEntryId && state.nextEntryId <= kMaxEntries + 1, "Invalid next timetable entry ID");

            const auto serviceCount = input.read<uint32_t>();
            require(serviceCount <= Limits::kMaxVehicles && serviceCount <= input.remaining() / kMinServiceSize, "Invalid timetable service count");
            state.services.reserve(serviceCount);
            std::vector<bool> seenEntryIds(state.nextEntryId);
            size_t totalEntries = 0;
            uint32_t previousServiceId{};
            for (uint32_t i = 0; i < serviceCount; ++i)
            {
                Vehicles::TimetableManager::Service service;
                service.id = input.read<uint32_t>();
                service.revision = input.read<uint32_t>();
                require(i == 0 || previousServiceId < service.id, "Non-canonical timetable service order");
                previousServiceId = service.id;

                const auto entryCount = input.read<uint16_t>();
                require(entryCount <= Limits::kMaxOrdersPerVehicle && entryCount <= kMaxEntries - totalEntries && entryCount <= input.remaining() / kMinEntrySize, "Invalid timetable entry count");
                totalEntries += entryCount;
                service.entries.reserve(entryCount);
                uint8_t previousOrderIndex{};
                for (uint16_t j = 0; j < entryCount; ++j)
                {
                    Vehicles::TimetableManager::TimetableEntry entry;
                    entry.id = input.read<uint32_t>();
                    entry.orderIndex = input.read<uint8_t>();
                    entry.orderType = static_cast<Vehicles::OrderType>(input.read<uint8_t>());
                    entry.station = static_cast<StationId>(input.read<uint16_t>());
                    const auto entryFlags = input.read<uint8_t>();
                    require((entryFlags & ~kKnownTimetableEntryFlags) == 0, "Invalid timetable entry flags");
                    require(j == 0 || previousOrderIndex < entry.orderIndex, "Non-canonical timetable entry order");
                    require(entry.id != Vehicles::TimetableManager::kInvalidEntryId && entry.id < seenEntryIds.size() && !seenEntryIds[entry.id], "Duplicate or invalid timetable entry ID");
                    previousOrderIndex = entry.orderIndex;
                    seenEntryIds[entry.id] = true;
                    if ((entryFlags & kTimetableEntryHasTravel) != 0)
                    {
                        entry.travelMinutes = input.read<uint32_t>();
                    }
                    if ((entryFlags & kTimetableEntryHasDwell) != 0)
                    {
                        entry.dwellMinutes = input.read<uint32_t>();
                    }
                    if ((entryFlags & kTimetableEntryHasDispatch) != 0)
                    {
                        Vehicles::TimetableManager::DispatchPattern dispatch;
                        const auto dispatchFlags = input.read<uint8_t>();
                        require((dispatchFlags & ~kKnownDispatchFlags) == 0, "Invalid timetable dispatch flags");
                        dispatch.periodMinutes = input.read<uint32_t>();
                        dispatch.phaseMinutes = input.read<uint32_t>();
                        dispatch.maxDelayMinutes = input.read<uint32_t>();
                        const auto slotCount = input.read<uint16_t>();
                        if ((dispatchFlags & kDispatchHasLastClaimedMinute) != 0)
                        {
                            dispatch.lastClaimedMinute = input.readSigned();
                        }
                        require(slotCount <= Vehicles::TimetableManager::kMaxSlots && slotCount <= input.remaining() / sizeof(uint32_t), "Invalid timetable dispatch slot count");
                        dispatch.slots.reserve(slotCount);
                        uint32_t previousSlot{};
                        for (uint16_t k = 0; k < slotCount; ++k)
                        {
                            const auto slot = input.read<uint32_t>();
                            require(k == 0 || previousSlot < slot, "Non-canonical timetable dispatch slots");
                            previousSlot = slot;
                            dispatch.slots.push_back(slot);
                        }
                        entry.dispatch = std::move(dispatch);
                    }
                    service.entries.push_back(std::move(entry));
                }
                state.services.push_back(std::move(service));
            }

            const auto assignmentCount = input.read<uint32_t>();
            require(assignmentCount <= Limits::kMaxEntities && assignmentCount <= input.remaining() / kAssignmentSize, "Invalid timetable assignment count");
            state.assignments.reserve(assignmentCount);
            uint16_t previousAssignment{};
            for (uint32_t i = 0; i < assignmentCount; ++i)
            {
                Vehicles::TimetableManager::VehicleAssignment assignment;
                const auto vehicle = input.read<uint16_t>();
                assignment.vehicle = static_cast<EntityId>(vehicle);
                assignment.service = input.read<uint32_t>();
                require(i == 0 || previousAssignment < vehicle, "Non-canonical timetable assignment order");
                previousAssignment = vehicle;
                state.assignments.push_back(assignment);
            }

            const auto runtimeCount = input.read<uint32_t>();
            require(runtimeCount <= Limits::kMaxEntities && runtimeCount <= input.remaining() / kMinRuntimeSize, "Invalid timetable runtime count");
            state.vehicles.reserve(runtimeCount);
            uint16_t previousRuntime{};
            for (uint32_t i = 0; i < runtimeCount; ++i)
            {
                Vehicles::TimetableManager::VehicleRuntime runtime;
                const auto vehicle = input.read<uint16_t>();
                runtime.vehicle = static_cast<EntityId>(vehicle);
                runtime.service = input.read<uint32_t>();
                runtime.serviceRevision = input.read<uint32_t>();
                runtime.currentEntry = input.read<uint32_t>();
                runtime.scheduledArrivalTick = input.read<uint64_t>();
                runtime.scheduledDepartureTick = input.read<uint64_t>();
                const auto runtimeFlags = input.read<uint8_t>();
                require((runtimeFlags & ~kKnownRuntimeFlags) == 0, "Invalid timetable runtime flags");
                require(i == 0 || previousRuntime < vehicle, "Non-canonical timetable runtime order");
                previousRuntime = vehicle;
                if ((runtimeFlags & kRuntimeHasAssignedSlot) != 0)
                {
                    runtime.assignedSlotMinute = input.readSigned();
                }
                runtime.latenessTicks = input.readSigned();
                runtime.timetableStarted = (runtimeFlags & kRuntimeTimetableStarted) != 0;
                runtime.atTimedStop = (runtimeFlags & kRuntimeAtTimedStop) != 0;
                runtime.released = (runtimeFlags & kRuntimeReleased) != 0;
                runtime.waiting = (runtimeFlags & kRuntimeWaiting) != 0;
                state.vehicles.push_back(runtime);
            }

            require(input.empty(), "Trailing timetable data");
            require(Vehicles::TimetableManager::validateState(state), "Invalid timetable state");
            return state;
        }

        std::vector<std::byte> encodeStationTileOverflow(const std::vector<StationTileOverflow>& stations)
        {
            require(!stations.empty(), "Station tile overflow must not be empty");

            std::vector<StationTileOverflow> sorted = stations;
            std::ranges::sort(sorted, {}, &StationTileOverflow::station);
            Writer payload;
            payload.write(static_cast<uint32_t>(sorted.size()));
            uint16_t previousStation{};
            bool hasPreviousStation = false;
            for (const auto& entry : sorted)
            {
                const auto station = static_cast<uint16_t>(entry.station);
                require(station < Limits::kMaxStations, "Invalid station tile overflow ID");
                require(!hasPreviousStation || previousStation < station, "Duplicate station tile overflow ID");
                previousStation = station;
                hasPreviousStation = true;
                require(entry.stationTileSize > kMaxStationTilesInSave && entry.stationTileSize <= kMaxStationTiles, "Invalid station tile overflow size");
                require(entry.stationTiles.size() == entry.stationTileSize, "Invalid station tile overflow tile count");
                payload.write(station);
                payload.write(entry.stationTileSize);
                for (const auto& pos : entry.stationTiles)
                {
                    require(World::validCoords(World::Pos2{ pos.x, pos.y }), "Invalid station tile overflow coordinates");
                    require(pos.z >= 0, "Invalid station tile overflow height");
                    payload.write(static_cast<uint16_t>(pos.x));
                    payload.write(static_cast<uint16_t>(pos.y));
                    payload.write(static_cast<uint16_t>(pos.z));
                }
            }
            return payload.take();
        }

        std::vector<StationTileOverflow> decodeStationTileOverflow(std::span<const std::byte> data)
        {
            Reader input(data);
            const auto stationCount = input.read<uint32_t>();
            require(stationCount != 0 && stationCount <= Limits::kMaxStations, "Invalid station tile overflow count");

            std::vector<StationTileOverflow> stations;
            stations.reserve(stationCount);
            uint16_t previousStation{};
            bool hasPreviousStation = false;
            for (uint32_t i = 0; i < stationCount; ++i)
            {
                const auto station = input.read<uint16_t>();
                require(station < Limits::kMaxStations, "Invalid station tile overflow ID");
                require(!hasPreviousStation || previousStation < station, "Non-canonical station tile overflow order");
                previousStation = station;
                hasPreviousStation = true;

                const auto stationTileSize = input.read<uint16_t>();
                require(stationTileSize > kMaxStationTilesInSave && stationTileSize <= kMaxStationTiles, "Invalid station tile overflow size");
                require(stationTileSize <= input.remaining() / (sizeof(uint16_t) * 3), "Truncated station tile overflow");

                auto& entry = stations.emplace_back();
                entry.station = static_cast<StationId>(station);
                entry.stationTileSize = stationTileSize;
                entry.stationTiles.reserve(stationTileSize);
                for (uint16_t j = 0; j < stationTileSize; ++j)
                {
                    const auto x = static_cast<int16_t>(input.read<uint16_t>());
                    const auto y = static_cast<int16_t>(input.read<uint16_t>());
                    const auto z = static_cast<int16_t>(input.read<uint16_t>());
                    require(World::validCoords(World::Pos2{ x, y }), "Invalid station tile overflow coordinates");
                    require(z >= 0, "Invalid station tile overflow height");
                    entry.stationTiles.emplace_back(x, y, z);
                }
            }
            require(input.empty(), "Trailing station tile overflow data");
            return stations;
        }

        std::vector<std::byte> encodeGameRules(const GameRules::State& state)
        {
            constexpr uint8_t kVehiclesNeverExpire = 1U << 0;
            constexpr uint8_t kExtendedVehicleObjects = 1U << 1;
            Writer payload;
            const auto rules = static_cast<uint8_t>((state.vehiclesNeverExpire ? kVehiclesNeverExpire : 0)
                | (state.extendedVehicleObjects ? kExtendedVehicleObjects : 0));
            payload.write(rules);
            return payload.take();
        }

        GameRules::State decodeGameRules(std::span<const std::byte> data)
        {
            constexpr uint8_t kVehiclesNeverExpire = 1U << 0;
            constexpr uint8_t kExtendedVehicleObjects = 1U << 1;
            constexpr uint8_t kKnownRules = kVehiclesNeverExpire | kExtendedVehicleObjects;
            require(data.size() == sizeof(uint8_t), "Invalid game rules data size");
            Reader input(data);
            const auto rules = input.read<uint8_t>();
            require((rules & ~kKnownRules) == 0, "Unknown game rule bits");
            return {
                .vehiclesNeverExpire = (rules & kVehiclesNeverExpire) != 0,
                .extendedVehicleObjects = (rules & kExtendedVehicleObjects) != 0,
            };
        }

        std::vector<std::byte> encodeVehicleObjects(const VehicleObjectState& state)
        {
            require(!state.objects.empty() && state.objects.size() <= kExtendedVehicleObjectCount, "Invalid extended vehicle object count");
            auto objects = state.objects;
            std::ranges::sort(objects, {}, &VehicleObjectSlot::slot);

            std::array<bool, kExtendedVehicleObjectCount> occupied{};
            uint16_t previousSlot{};
            bool hasPreviousSlot = false;
            for (const auto& object : objects)
            {
                require(object.slot >= kExtendedVehicleObjectStart && object.slot < OpenLoco::Limits::kMaxVehicleObjects, "Invalid extended vehicle object slot");
                require(!hasPreviousSlot || previousSlot < object.slot, "Duplicate extended vehicle object slot");
                require(!object.header.isEmpty() && object.header.getType() == ObjectType::vehicle, "Invalid extended vehicle object header");
                occupied[object.slot - kExtendedVehicleObjectStart] = true;
                previousSlot = object.slot;
                hasPreviousSlot = true;
            }

            for (const auto& unlocks : state.companyUnlocks)
            {
                size_t validBitCount = 0;
                for (size_t vehicle = 0; vehicle < kExtendedVehicleObjectCount; ++vehicle)
                {
                    if (!unlocks[vehicle])
                    {
                        continue;
                    }
                    ++validBitCount;
                    require(occupied[vehicle], "Extended vehicle unlock refers to an empty slot");
                }
                require(unlocks.count() == validBitCount, "Non-zero extended vehicle unlock padding");
            }

            Writer payload;
            payload.write(static_cast<uint16_t>(objects.size()));
            payload.write(static_cast<uint16_t>(state.companyUnlocks.size()));
            payload.write(static_cast<uint16_t>(kVehicleUnlockWordCount));
            for (const auto& object : objects)
            {
                payload.write(object.slot);
                payload.write(object.header.flags);
                payload.writeBytes(std::as_bytes(std::span(object.header.name)));
                payload.write(object.header.checksum);
            }
            for (const auto& unlocks : state.companyUnlocks)
            {
                for (size_t wordIndex = 0; wordIndex < kVehicleUnlockWordCount; ++wordIndex)
                {
                    uint64_t word = 0;
                    for (size_t bit = 0; bit < kVehicleUnlockWordBits; ++bit)
                    {
                        const auto vehicle = wordIndex * kVehicleUnlockWordBits + bit;
                        if (vehicle < kExtendedVehicleObjectCount && unlocks[vehicle])
                        {
                            word |= uint64_t{ 1 } << bit;
                        }
                    }
                    payload.write(word);
                }
            }
            return payload.take();
        }

        VehicleObjectState decodeVehicleObjects(std::span<const std::byte> data)
        {
            Reader input(data);
            const auto objectCount = input.read<uint16_t>();
            const auto companyCount = input.read<uint16_t>();
            const auto unlockWordCount = input.read<uint16_t>();
            require(objectCount != 0 && objectCount <= kExtendedVehicleObjectCount, "Invalid extended vehicle object count");
            require(companyCount == OpenLoco::S5::Limits::kMaxCompanies, "Invalid extended vehicle company count");
            require(unlockWordCount == kVehicleUnlockWordCount, "Invalid extended vehicle unlock dimensions");

            VehicleObjectState state;
            state.objects.reserve(objectCount);
            std::array<bool, kExtendedVehicleObjectCount> occupied{};
            uint16_t previousSlot{};
            for (uint16_t i = 0; i < objectCount; ++i)
            {
                VehicleObjectSlot object;
                object.slot = input.read<uint16_t>();
                object.header.flags = input.read<uint32_t>();
                const auto name = input.readBytes(std::size(object.header.name));
                std::memcpy(object.header.name, name.data(), name.size());
                object.header.checksum = input.read<uint32_t>();
                require(object.slot >= kExtendedVehicleObjectStart && object.slot < OpenLoco::Limits::kMaxVehicleObjects, "Invalid extended vehicle object slot");
                require(i == 0 || previousSlot < object.slot, "Non-canonical extended vehicle object order");
                require(!object.header.isEmpty() && object.header.getType() == ObjectType::vehicle, "Invalid extended vehicle object header");
                occupied[object.slot - kExtendedVehicleObjectStart] = true;
                previousSlot = object.slot;
                state.objects.push_back(object);
            }

            constexpr auto kLastWordBits = kExtendedVehicleObjectCount % kVehicleUnlockWordBits;
            constexpr auto kLastWordMask = (uint64_t{ 1 } << kLastWordBits) - 1;
            for (auto& unlocks : state.companyUnlocks)
            {
                for (size_t wordIndex = 0; wordIndex < kVehicleUnlockWordCount; ++wordIndex)
                {
                    const auto word = input.read<uint64_t>();
                    if (wordIndex == kVehicleUnlockWordCount - 1)
                    {
                        require((word & ~kLastWordMask) == 0, "Non-zero extended vehicle unlock padding");
                    }
                    for (size_t bit = 0; bit < kVehicleUnlockWordBits; ++bit)
                    {
                        const auto vehicle = wordIndex * kVehicleUnlockWordBits + bit;
                        if (vehicle >= kExtendedVehicleObjectCount || (word & (uint64_t{ 1 } << bit)) == 0)
                        {
                            continue;
                        }
                        require(occupied[vehicle], "Extended vehicle unlock refers to an empty slot");
                        unlocks.set(vehicle, true);
                    }
                }
            }
            require(input.empty(), "Trailing extended vehicle object data");
            return state;
        }

        bool hasMagic(std::span<const std::byte> data, std::span<const std::byte, 8> magic)
        {
            return data.size() >= magic.size() && std::ranges::equal(data.first(magic.size()), magic);
        }
    }

    std::vector<std::byte> encode(const State& state)
    {
        return encode(StateView{
            .cargoDistState = state.cargoDistState ? &*state.cargoDistState : nullptr,
            .sharedOrderState = state.sharedOrderState ? &*state.sharedOrderState : nullptr,
            .pathReservationState = state.pathReservationState ? &*state.pathReservationState : nullptr,
            .vehicleAutoRenewalState = state.vehicleAutoRenewalState ? &*state.vehicleAutoRenewalState : nullptr,
            .vehicleReplacementState = state.vehicleReplacementState ? &*state.vehicleReplacementState : nullptr,
            .discardPathReservationsOnLoad = state.discardPathReservationsOnLoad,
            .railTrafficState = state.railTrafficState ? &*state.railTrafficState : nullptr,
            .stationTileOverflowState = state.stationTileOverflowState ? &*state.stationTileOverflowState : nullptr,
            .gameRulesState = state.gameRulesState ? &*state.gameRulesState : nullptr,
            .vehicleObjectState = state.vehicleObjectState ? &*state.vehicleObjectState : nullptr,
            .timetableState = state.timetableState ? &*state.timetableState : nullptr,
        });
    }

    std::vector<std::byte> encode(const StateView state)
    {
        Writer payload;
        if (state.cargoDistState != nullptr)
        {
            const auto cargoDist = CargoDist::encodeState(*state.cargoDistState);
            appendSection(payload, kCargoDistTag, cargoDist);
        }
        if (state.sharedOrderState != nullptr)
        {
            const auto sharedOrders = encodeSharedOrders(*state.sharedOrderState);
            appendSection(payload, kSharedOrdersTag, sharedOrders, kSectionRequired);
        }
        if (state.pathReservationState != nullptr)
        {
            const auto hasContinuations = std::ranges::any_of(state.pathReservationState->continuations, [](const auto& continuation) { return !continuation.empty(); });
            require(!state.discardPathReservationsOnLoad || !hasContinuations, "Legacy path reservations cannot contain continuations");
            const auto pathReservations = encodePathReservations(*state.pathReservationState);
            const auto version = state.discardPathReservationsOnLoad
                ? kSectionVersion
                : hasContinuations ? kPathReservationContinuationsSectionVersion
                                   : kPathReservationsSectionVersion;
            appendSection(payload, kPathReservationsTag, pathReservations, kSectionRequired, version);
        }
        if (state.vehicleAutoRenewalState != nullptr)
        {
            const auto vehicleAutoRenewal = encodeVehicleAutoRenewal(*state.vehicleAutoRenewalState);
            appendSection(payload, kVehicleAutoRenewalTag, vehicleAutoRenewal, kSectionRequired);
        }
        if (state.vehicleReplacementState != nullptr)
        {
            appendSection(payload, kVehicleReplacementTag, encodeVehicleReplacement(*state.vehicleReplacementState), kSectionRequired, kVehicleReplacementSectionVersion);
        }
        if (state.railTrafficState != nullptr)
        {
            const auto railTraffic = encodeRailTraffic(*state.railTrafficState);
            appendSection(payload, kRailTrafficTag, railTraffic, kSectionRequired);
        }
        if (state.stationTileOverflowState != nullptr)
        {
            const auto stationTileOverflow = encodeStationTileOverflow(*state.stationTileOverflowState);
            appendSection(payload, kStationTileOverflowTag, stationTileOverflow, kSectionRequired);
        }
        if (state.gameRulesState != nullptr)
        {
            appendSection(payload, kGameRulesTag, encodeGameRules(*state.gameRulesState), kSectionRequired);
        }
        if (state.vehicleObjectState != nullptr)
        {
            appendSection(payload, kVehicleObjectsTag, encodeVehicleObjects(*state.vehicleObjectState), kSectionRequired);
        }
        if (state.timetableState != nullptr)
        {
            appendSection(payload, kTimetableTag, encodeTimetable(*state.timetableState), kSectionRequired);
        }

        require(payload.size() <= std::numeric_limits<uint32_t>::max(), "Save extension is too large");
        Writer output;
        output.writeBytes(kMagic);
        output.write(kVersion);
        output.write(kHeaderSize);
        output.write(static_cast<uint32_t>(payload.size()));
        output.writeBytes(payload.data());
        return output.take();
    }

    State decode(const std::span<const std::byte> data)
    {
        require(data.size() <= kMaxDataSize, "Save extension is too large");
        if (hasMagic(data, kLegacyCargoDistMagic))
        {
            State state;
            state.cargoDistState = CargoDist::decodeState(data);
            return state;
        }

        Reader input(data);
        require(std::ranges::equal(input.readBytes(kMagic.size()), kMagic), "Invalid save extension magic");
        require(input.read<uint16_t>() == kVersion, "Unsupported save extension version");
        require(input.read<uint16_t>() == kHeaderSize, "Invalid save extension header");
        const auto payloadSize = input.read<uint32_t>();
        require(payloadSize == input.remaining(), "Invalid save extension payload size");
        Reader sections(input.readBytes(payloadSize));
        require(input.empty(), "Trailing save extension data");

        State state;
        bool hasCargoDist = false;
        bool hasSharedOrders = false;
        bool hasPathReservations = false;
        bool hasVehicleAutoRenewal = false;
        bool hasRailTraffic = false;
        bool hasVehicleReplacement = false;
        bool hasStationTileOverflow = false;
        bool hasGameRules = false;
        bool hasVehicleObjects = false;
        bool hasTimetable = false;
        while (!sections.empty())
        {
            const auto tag = sections.readBytes(4);
            const auto version = sections.read<uint16_t>();
            const auto flags = sections.read<uint16_t>();
            const auto sectionData = sections.readBytes(sections.read<uint32_t>());

            if (std::ranges::equal(tag, kCargoDistTag))
            {
                require((flags & ~kKnownSectionFlags) == 0, "Invalid CargoDist section flags");
                require(!hasCargoDist, "Duplicate CargoDist save extension section");
                hasCargoDist = true;
                require(version == kSectionVersion, "Unsupported CargoDist section version");
                state.cargoDistState = CargoDist::decodeState(sectionData);
            }
            else if (std::ranges::equal(tag, kSharedOrdersTag))
            {
                require((flags & ~kKnownSectionFlags) == 0, "Invalid shared order section flags");
                require(!hasSharedOrders, "Duplicate shared order save extension section");
                hasSharedOrders = true;
                require(version == kSectionVersion, "Unsupported shared order section version");
                state.sharedOrderState = decodeSharedOrders(sectionData);
            }
            else if (std::ranges::equal(tag, kPathReservationsTag))
            {
                require((flags & ~kKnownSectionFlags) == 0, "Invalid path reservation section flags");
                require(!hasPathReservations, "Duplicate path reservation save extension section");
                hasPathReservations = true;
                require(version == kSectionVersion || version == kPathReservationsSectionVersion || version == kPathReservationContinuationsSectionVersion, "Unsupported path reservation section version");
                state.pathReservationState = decodePathReservations(sectionData, version);
                state.discardPathReservationsOnLoad = version == kSectionVersion;
            }
            else if (std::ranges::equal(tag, kVehicleAutoRenewalTag))
            {
                require((flags & ~kKnownSectionFlags) == 0, "Invalid vehicle auto-renewal section flags");
                require(!hasVehicleAutoRenewal, "Duplicate vehicle auto-renewal save extension section");
                hasVehicleAutoRenewal = true;
                require(version == kSectionVersion, "Unsupported vehicle auto-renewal section version");
                state.vehicleAutoRenewalState = decodeVehicleAutoRenewal(sectionData);
            }
            else if (std::ranges::equal(tag, kRailTrafficTag))
            {
                require((flags & ~kKnownSectionFlags) == 0, "Invalid rail traffic section flags");
                require(!hasRailTraffic, "Duplicate rail traffic save extension section");
                hasRailTraffic = true;
                require(version == kSectionVersion, "Unsupported rail traffic section version");
                state.railTrafficState = decodeRailTraffic(sectionData);
            }
            else if (std::ranges::equal(tag, kVehicleReplacementTag))
            {
                require((flags & ~kKnownSectionFlags) == 0 && !hasVehicleReplacement, "Invalid vehicle replacement section");
                require(version == kSectionVersion || version == kVehicleReplacementPendingPlacementsVersion || version == kVehicleReplacementSectionVersion, "Unsupported vehicle replacement section version");
                hasVehicleReplacement = true;
                state.vehicleReplacementState = decodeVehicleReplacement(sectionData, version);
            }
            else if (std::ranges::equal(tag, kStationTileOverflowTag))
            {
                require((flags & ~kKnownSectionFlags) == 0, "Invalid station tile overflow section flags");
                require(!hasStationTileOverflow, "Duplicate station tile overflow save extension section");
                hasStationTileOverflow = true;
                require(version == kSectionVersion, "Unsupported station tile overflow section version");
                state.stationTileOverflowState = decodeStationTileOverflow(sectionData);
            }
            else if (std::ranges::equal(tag, kGameRulesTag))
            {
                require(flags == kSectionRequired, "Invalid game rules section flags");
                require(!hasGameRules, "Duplicate game rules save extension section");
                hasGameRules = true;
                require(version == kSectionVersion, "Unsupported game rules section version");
                state.gameRulesState = decodeGameRules(sectionData);
            }
            else if (std::ranges::equal(tag, kVehicleObjectsTag))
            {
                require(flags == kSectionRequired, "Invalid extended vehicle object section flags");
                require(!hasVehicleObjects, "Duplicate extended vehicle object save extension section");
                hasVehicleObjects = true;
                require(version == kSectionVersion, "Unsupported extended vehicle object section version");
                state.vehicleObjectState = decodeVehicleObjects(sectionData);
            }
            else if (std::ranges::equal(tag, kTimetableTag))
            {
                require(flags == kSectionRequired, "Invalid timetable section flags");
                require(!hasTimetable, "Duplicate timetable save extension section");
                hasTimetable = true;
                require(version == kSectionVersion, "Unsupported timetable section version");
                state.timetableState = decodeTimetable(sectionData);
            }
            else
            {
                require((flags & kSectionRequired) == 0, "Unknown required save extension section");
            }
        }
        return state;
    }
}
