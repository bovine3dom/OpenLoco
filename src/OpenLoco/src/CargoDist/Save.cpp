// SPDX-License-Identifier: MIT
#include <OpenLoco/CargoDist/Save.h>

#include "S5/Limits.h"
#include <algorithm>
#include <array>
#include <bit>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace OpenLoco::CargoDist
{
    namespace
    {
        constexpr std::array<std::byte, 8> kMagic = {
            std::byte{ 'O' },
            std::byte{ 'L' },
            std::byte{ 'C' },
            std::byte{ 'D' },
            std::byte{ 'I' },
            std::byte{ 'S' },
            std::byte{ 'T' },
            std::byte{ 0 },
        };
        constexpr uint16_t kVersion = 1;
        constexpr uint16_t kHeaderSize = 16;
        constexpr uint32_t kMaxStationLists = S5::Limits::kMaxStations * S5::Limits::kMaxCargoObjects;
        constexpr uint32_t kMaxVehicleLists = S5::Limits::kMaxEntities * 2;
        constexpr uint32_t kMaxFlowLists = 1'000'000;

        class Encoder
        {
        public:
            template<typename T>
            void write(T value)
            {
                static_assert(std::is_unsigned_v<T>);
                for (size_t i = 0; i < sizeof(T); ++i)
                {
                    if (_data.size() == kMaxSaveDataSize)
                    {
                        throw std::runtime_error("CargoDist save data is too large");
                    }
                    _data.push_back(static_cast<std::byte>(value & 0xFF));
                    value >>= 8;
                }
            }

            void write(int64_t value)
            {
                write(std::bit_cast<uint64_t>(value));
            }

            void writeBytes(std::span<const std::byte> bytes)
            {
                if (bytes.size() > kMaxSaveDataSize - _data.size())
                {
                    throw std::runtime_error("CargoDist save data is too large");
                }
                _data.insert(_data.end(), bytes.begin(), bytes.end());
            }

            const std::vector<std::byte>& data() const { return _data; }

        private:
            std::vector<std::byte> _data;
        };

        class Decoder
        {
        public:
            explicit Decoder(std::span<const std::byte> data)
                : _data(data)
            {
            }

            template<typename T>
            T read()
            {
                static_assert(std::is_unsigned_v<T>);
                require(sizeof(T));
                T value = 0;
                for (size_t i = 0; i < sizeof(T); ++i)
                {
                    value |= static_cast<T>(std::to_integer<uint8_t>(_data[_position++])) << (i * 8);
                }
                return value;
            }

            int64_t readInt64()
            {
                return std::bit_cast<int64_t>(read<uint64_t>());
            }

            std::span<const std::byte> readBytes(size_t size)
            {
                require(size);
                const auto result = _data.subspan(_position, size);
                _position += size;
                return result;
            }

            bool empty() const { return _position == _data.size(); }
            size_t remaining() const { return _data.size() - _position; }

        private:
            void require(size_t size) const
            {
                if (size > remaining())
                {
                    throw std::runtime_error("Truncated CargoDist save data");
                }
            }

            std::span<const std::byte> _data;
            size_t _position{};
        };

        uint16_t stationValue(StationId station)
        {
            return static_cast<uint16_t>(station);
        }

        uint16_t entityValue(EntityId entity)
        {
            return static_cast<uint16_t>(entity);
        }

        bool isValidStation(StationId station, bool allowNull = false)
        {
            return (allowNull && station == StationId::null) || stationValue(station) < S5::Limits::kMaxStations;
        }

        void require(bool condition, const char* message)
        {
            if (!condition)
            {
                throw std::runtime_error(message);
            }
        }

        void encodePackets(Encoder& encoder, const PacketList& packets, uint32_t maxQuantity)
        {
            require(packets.size() <= std::numeric_limits<uint32_t>::max(), "Too many CargoDist packets");
            require(packets.quantity() <= maxQuantity, "CargoDist packet quantity exceeds native capacity");
            encoder.write(static_cast<uint32_t>(packets.size()));
            for (const auto& packet : packets.packets())
            {
                require(packet.quantity != 0, "CargoDist packet has zero quantity");
                require(isValidStation(packet.origin), "CargoDist packet has invalid origin");
                require(isValidStation(packet.nextHop, true), "CargoDist packet has invalid next hop");
                encoder.write(packet.quantity);
                encoder.write(stationValue(packet.origin));
                encoder.write(stationValue(packet.nextHop));
                encoder.write(packet.age);
                encoder.write<uint8_t>(0);
            }
        }

        PacketList decodePackets(Decoder& decoder, uint32_t maxQuantity)
        {
            const auto count = decoder.read<uint32_t>();
            require(count <= maxQuantity, "Too many CargoDist packets");
            PacketList::Container packets;
            packets.reserve(count);
            uint32_t quantity = 0;
            for (uint32_t i = 0; i < count; ++i)
            {
                CargoPacket packet;
                packet.quantity = decoder.read<uint16_t>();
                packet.origin = StationId(decoder.read<uint16_t>());
                packet.nextHop = StationId(decoder.read<uint16_t>());
                packet.age = decoder.read<uint8_t>();
                decoder.read<uint8_t>();
                require(packet.quantity != 0, "CargoDist packet has zero quantity");
                require(isValidStation(packet.origin), "CargoDist packet has invalid origin");
                require(isValidStation(packet.nextHop, true), "CargoDist packet has invalid next hop");
                require(packet.quantity <= maxQuantity - quantity, "CargoDist packet quantity exceeds native capacity");
                quantity += packet.quantity;
                packets.push_back(packet);
            }
            return PacketList::fromPackets(std::move(packets));
        }

        void validateFlowOptions(const std::vector<FlowOption>& options)
        {
            uint64_t totalWeight = 0;
            for (const auto& option : options)
            {
                require(isValidStation(option.via) && option.weight != 0, "Invalid CargoDist flow option");
                totalWeight += option.weight;
            }
            require(totalWeight <= std::numeric_limits<uint32_t>::max(), "CargoDist flow weight exceeds supported range");

            int64_t currentTotal = 0;
            const auto limit = static_cast<int64_t>(totalWeight);
            StationId previousVia = StationId::null;
            for (const auto& option : options)
            {
                require(option.current >= -limit && option.current <= limit, "Invalid CargoDist flow cursor");
                require(previousVia == StationId::null || stationValue(previousVia) < stationValue(option.via), "Unsorted CargoDist flow options");
                currentTotal += option.current;
                previousVia = option.via;
            }
            require(currentTotal == 0, "Unbalanced CargoDist flow cursors");
        }

        template<typename TMap, typename TPredicate>
        uint32_t countMatching(const TMap& map, TPredicate&& predicate)
        {
            return static_cast<uint32_t>(std::count_if(map.begin(), map.end(), [&](const auto& item) { return predicate(item); }));
        }
    }

    std::vector<std::byte> encodeState(const State& state)
    {
        Encoder payload;
        for (const auto mode : state.settings.modes)
        {
            require(mode == DistributionMode::manual || mode == DistributionMode::asymmetric, "Invalid CargoDist mode");
            payload.write(static_cast<uint8_t>(mode));
        }
        payload.write(state.settings.routing.distanceEffect);
        payload.write(state.settings.routing.saturation);
        payload.write(state.settings.routing.accuracy);
        payload.write<uint8_t>(0);
        payload.write(state.settings.recalculationInterval);
        payload.write<uint16_t>(0);
        payload.write(state.nextRecalculationDay);
        payload.write(static_cast<uint8_t>(state.graphDirty));
        payload.write<uint8_t>(0);
        payload.write<uint16_t>(0);

        payload.write(countMatching(state.stationCargo, [](const auto& item) { return !item.second.empty(); }));
        for (const auto& [key, packets] : state.stationCargo)
        {
            if (packets.empty())
            {
                continue;
            }
            require(key.cargo < state.settings.modes.size() && isValidStation(key.station), "Invalid CargoDist station cargo key");
            payload.write(stationValue(key.station));
            payload.write(key.cargo);
            payload.write<uint8_t>(0);
            encodePackets(payload, packets, std::numeric_limits<uint16_t>::max());
        }

        payload.write(countMatching(state.vehicleCargo, [](const auto& item) { return !item.second.empty(); }));
        for (const auto& [key, packets] : state.vehicleCargo)
        {
            if (packets.empty())
            {
                continue;
            }
            require(entityValue(key.component) < S5::Limits::kMaxEntities, "Invalid CargoDist vehicle cargo key");
            require(key.slot == VehicleCargoSlot::primary || key.slot == VehicleCargoSlot::secondary, "Invalid CargoDist vehicle cargo slot");
            payload.write(entityValue(key.component));
            payload.write(static_cast<uint8_t>(key.slot));
            payload.write<uint8_t>(0);
            encodePackets(payload, packets, std::numeric_limits<uint8_t>::max());
        }

        payload.write(countMatching(state.supply, [](const auto& item) { return item.second != 0; }));
        for (const auto& [key, amount] : state.supply)
        {
            if (amount == 0)
            {
                continue;
            }
            require(key.first < state.settings.modes.size() && isValidStation(key.second), "Invalid CargoDist supply key");
            payload.write(key.first);
            payload.write<uint8_t>(0);
            payload.write(stationValue(key.second));
            payload.write(amount);
        }

        payload.write(countMatching(state.flows, [](const auto& item) { return !item.second.empty(); }));
        for (const auto& [key, options] : state.flows)
        {
            if (options.empty())
            {
                continue;
            }
            require(key.cargo < state.settings.modes.size() && isValidStation(key.station) && isValidStation(key.origin), "Invalid CargoDist flow key");
            require(options.size() <= S5::Limits::kMaxStations, "Too many CargoDist flow options");
            validateFlowOptions(options);
            payload.write(key.cargo);
            payload.write<uint8_t>(0);
            payload.write(stationValue(key.station));
            payload.write(stationValue(key.origin));
            payload.write(static_cast<uint16_t>(options.size()));
            for (const auto& option : options)
            {
                payload.write(stationValue(option.via));
                payload.write<uint16_t>(0);
                payload.write(option.weight);
                payload.write(option.current);
            }
        }

        require(payload.data().size() <= kMaxSaveDataSize - kHeaderSize, "CargoDist save data is too large");
        Encoder result;
        result.writeBytes(std::span{ kMagic });
        result.write(kVersion);
        result.write(kHeaderSize);
        result.write(static_cast<uint32_t>(payload.data().size()));
        result.writeBytes(std::span{ payload.data() });
        return result.data();
    }

    State decodeState(std::span<const std::byte> data)
    {
        require(data.size() <= kMaxSaveDataSize, "CargoDist save data is too large");
        Decoder decoder(data);
        require(std::ranges::equal(decoder.readBytes(kMagic.size()), kMagic), "Invalid CargoDist save magic");
        require(decoder.read<uint16_t>() == kVersion, "Unsupported CargoDist save version");
        require(decoder.read<uint16_t>() == kHeaderSize, "Invalid CargoDist save header");
        require(decoder.read<uint32_t>() == decoder.remaining(), "Invalid CargoDist save payload size");

        State state;
        for (auto& mode : state.settings.modes)
        {
            mode = static_cast<DistributionMode>(decoder.read<uint8_t>());
            require(mode == DistributionMode::manual || mode == DistributionMode::asymmetric, "Invalid CargoDist mode");
        }
        state.settings.routing.distanceEffect = decoder.read<uint8_t>();
        state.settings.routing.saturation = decoder.read<uint8_t>();
        state.settings.routing.accuracy = decoder.read<uint8_t>();
        decoder.read<uint8_t>();
        state.settings.recalculationInterval = decoder.read<uint16_t>();
        decoder.read<uint16_t>();
        state.nextRecalculationDay = decoder.read<uint32_t>();
        const auto graphDirty = decoder.read<uint8_t>();
        require(graphDirty <= 1, "Invalid CargoDist graph state");
        state.graphDirty = graphDirty != 0;
        decoder.read<uint8_t>();
        decoder.read<uint16_t>();

        const auto stationListCount = decoder.read<uint32_t>();
        require(stationListCount <= kMaxStationLists, "Too many CargoDist station cargo lists");
        for (uint32_t i = 0; i < stationListCount; ++i)
        {
            StationCargoKey key;
            key.station = StationId(decoder.read<uint16_t>());
            key.cargo = decoder.read<uint8_t>();
            decoder.read<uint8_t>();
            require(isValidStation(key.station) && key.cargo < state.settings.modes.size(), "Invalid CargoDist station cargo key");
            require(state.stationCargo.emplace(key, decodePackets(decoder, std::numeric_limits<uint16_t>::max())).second, "Duplicate CargoDist station cargo key");
        }

        const auto vehicleListCount = decoder.read<uint32_t>();
        require(vehicleListCount <= kMaxVehicleLists, "Too many CargoDist vehicle cargo lists");
        for (uint32_t i = 0; i < vehicleListCount; ++i)
        {
            VehicleCargoKey key;
            key.component = EntityId(decoder.read<uint16_t>());
            key.slot = static_cast<VehicleCargoSlot>(decoder.read<uint8_t>());
            decoder.read<uint8_t>();
            require(entityValue(key.component) < S5::Limits::kMaxEntities, "Invalid CargoDist vehicle cargo key");
            require(key.slot == VehicleCargoSlot::primary || key.slot == VehicleCargoSlot::secondary, "Invalid CargoDist vehicle cargo slot");
            require(state.vehicleCargo.emplace(key, decodePackets(decoder, std::numeric_limits<uint8_t>::max())).second, "Duplicate CargoDist vehicle cargo key");
        }

        const auto supplyCount = decoder.read<uint32_t>();
        require(supplyCount <= kMaxStationLists, "Too many CargoDist supply entries");
        for (uint32_t i = 0; i < supplyCount; ++i)
        {
            const auto cargo = decoder.read<uint8_t>();
            decoder.read<uint8_t>();
            const auto station = StationId(decoder.read<uint16_t>());
            const auto amount = decoder.read<uint32_t>();
            require(cargo < state.settings.modes.size() && isValidStation(station) && amount != 0, "Invalid CargoDist supply entry");
            require(state.supply.emplace(std::pair{ cargo, station }, amount).second, "Duplicate CargoDist supply entry");
        }

        const auto flowCount = decoder.read<uint32_t>();
        require(flowCount <= kMaxFlowLists, "Too many CargoDist flow entries");
        for (uint32_t i = 0; i < flowCount; ++i)
        {
            FlowKey key;
            key.cargo = decoder.read<uint8_t>();
            decoder.read<uint8_t>();
            key.station = StationId(decoder.read<uint16_t>());
            key.origin = StationId(decoder.read<uint16_t>());
            const auto optionCount = decoder.read<uint16_t>();
            require(key.cargo < state.settings.modes.size() && isValidStation(key.station) && isValidStation(key.origin), "Invalid CargoDist flow key");
            require(optionCount != 0 && optionCount <= S5::Limits::kMaxStations, "Invalid CargoDist flow option count");
            std::vector<FlowOption> options;
            options.reserve(optionCount);
            for (uint16_t j = 0; j < optionCount; ++j)
            {
                FlowOption option;
                option.via = StationId(decoder.read<uint16_t>());
                decoder.read<uint16_t>();
                option.weight = decoder.read<uint32_t>();
                option.current = decoder.readInt64();
                options.push_back(option);
            }
            validateFlowOptions(options);
            require(state.flows.emplace(key, std::move(options)).second, "Duplicate CargoDist flow key");
        }

        require(decoder.empty(), "Trailing CargoDist save data");
        return state;
    }
}
