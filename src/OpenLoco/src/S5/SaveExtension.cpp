// SPDX-License-Identifier: MIT
#include <OpenLoco/S5/SaveExtension.h>

#include "S5/Limits.h"
#include <OpenLoco/CargoDist/Save.h>
#include <algorithm>
#include <array>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

namespace OpenLoco::S5::SaveExtension
{
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
        constexpr uint16_t kVersion = 1;
        constexpr uint16_t kHeaderSize = 16;
        constexpr uint16_t kSectionVersion = 1;
        // Version 1 reservations may contain routes that bypass their active waypoint.
        constexpr uint16_t kPathReservationsSectionVersion = 2;
        constexpr uint16_t kSectionRequired = 1U << 0;
        constexpr uint16_t kKnownSectionFlags = kSectionRequired;
        constexpr size_t kSectionHeaderSize = 12;

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
            return payload.take();
        }

        Vehicles::RoutingManager::State decodePathReservations(std::span<const std::byte> data)
        {
            require(data.size() == Limits::kMaxVehicles * sizeof(uint64_t), "Invalid path reservation data size");
            Reader input(data);
            Vehicles::RoutingManager::State state;
            for (auto& mask : state.pathReservedRoutings)
            {
                mask = input.read<uint64_t>();
            }
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
            .discardPathReservationsOnLoad = state.discardPathReservationsOnLoad,
            .railTrafficState = state.railTrafficState ? &*state.railTrafficState : nullptr,
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
            const auto pathReservations = encodePathReservations(*state.pathReservationState);
            const auto version = state.discardPathReservationsOnLoad ? kSectionVersion : kPathReservationsSectionVersion;
            appendSection(payload, kPathReservationsTag, pathReservations, kSectionRequired, version);
        }
        if (state.vehicleAutoRenewalState != nullptr)
        {
            const auto vehicleAutoRenewal = encodeVehicleAutoRenewal(*state.vehicleAutoRenewalState);
            appendSection(payload, kVehicleAutoRenewalTag, vehicleAutoRenewal, kSectionRequired);
        }
        if (state.railTrafficState != nullptr)
        {
            const auto railTraffic = encodeRailTraffic(*state.railTrafficState);
            appendSection(payload, kRailTrafficTag, railTraffic, kSectionRequired);
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
                require(version == kSectionVersion || version == kPathReservationsSectionVersion, "Unsupported path reservation section version");
                state.pathReservationState = decodePathReservations(sectionData);
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
            else
            {
                require((flags & kSectionRequired) == 0, "Unknown required save extension section");
            }
        }
        return state;
    }
}
