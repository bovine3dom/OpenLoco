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
        constexpr uint16_t kVersion = 11;
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
                    _data.push_back(static_cast<std::byte>((value >> (i * 8)) & 0xFF));
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

        uint16_t serviceValue(ServiceId service)
        {
            return static_cast<uint16_t>(service);
        }

        bool isValidStation(StationId station, bool allowNull = false)
        {
            return (allowNull && station == StationId::null) || stationValue(station) < S5::Limits::kMaxStations;
        }

        bool isValidIndustry(IndustryId industry, bool allowNull = false)
        {
            return (allowNull && industry == IndustryId::null) || enumValue(industry) < S5::Limits::kMaxIndustries;
        }

        bool isValidTown(TownId town, bool allowNull = false)
        {
            return (allowNull && town == TownId::null) || enumValue(town) < S5::Limits::kMaxTowns;
        }

        void require(bool condition, const char* message)
        {
            if (!condition)
            {
                throw std::runtime_error(message);
            }
        }

        bool isPopulatedServicePoint(const ServicePoint& point)
        {
            return point.service != ServiceId::null
                && serviceValue(point.service) < S5::Limits::kMaxEntities
                && point.occurrence < S5::Limits::kMaxOrdersPerVehicle;
        }

        bool isValidServicePoint(const ServicePoint& point)
        {
            return point.empty() || isPopulatedServicePoint(point);
        }

        void validateServiceLeg(const ServicePoint& departure, const ServicePoint& arrival, bool local, const char* message)
        {
            const auto empty = departure.empty() && arrival.empty();
            const auto populated = isPopulatedServicePoint(departure) && isPopulatedServicePoint(arrival) && departure.service == arrival.service;
            require(isValidServicePoint(departure) && isValidServicePoint(arrival) && (empty || (!local && populated)), message);
        }

        void validatePacketMetadata(const CargoPacket& packet)
        {
            require(isValidStation(packet.legOrigin, true), "CargoDist packet has invalid leg origin");
            require(packet.tripKind == PassengerTripKind::ordinary || packet.tripKind == PassengerTripKind::holidayOutbound || packet.tripKind == PassengerTripKind::holidayReturn, "CargoDist packet has invalid trip kind");
            require(packet.tripKind != PassengerTripKind::ordinary || packet.holidayIndustry == IndustryId::null, "Ordinary CargoDist packet has holiday industry");
            require(packet.tripKind != PassengerTripKind::ordinary || packet.homeTown == TownId::null, "Ordinary CargoDist packet has holiday home");
            require(packet.tripKind == PassengerTripKind::ordinary || isValidIndustry(packet.holidayIndustry), "Holiday CargoDist packet has invalid industry");
            require(packet.tripKind == PassengerTripKind::ordinary || isValidTown(packet.homeTown), "Holiday CargoDist packet has invalid home");
        }

        void encodeServicePoint(Encoder& encoder, const ServicePoint& point)
        {
            encoder.write(serviceValue(point.service));
            encoder.write(point.occurrence);
        }

        ServicePoint decodeServicePoint(Decoder& decoder)
        {
            return { static_cast<ServiceId>(decoder.read<uint16_t>()), decoder.read<uint16_t>() };
        }

        void validateRevenueContributions(const std::span<const RevenueContribution> contributions)
        {
            require(contributions.size() <= S5::Limits::kMaxEntities, "Too many CargoDist revenue contributions");
            for (size_t i = 0; i < contributions.size(); ++i)
            {
                const auto& contribution = contributions[i];
                require(entityValue(contribution.vehicle) < S5::Limits::kMaxEntities && contribution.weight != 0, "Invalid CargoDist revenue contribution");
                require(i == 0 || contributions[i - 1].vehicle < contribution.vehicle, "Non-canonical CargoDist revenue contributions");
            }
        }

        void encodeRevenueContributions(Encoder& encoder, const std::span<const RevenueContribution> contributions)
        {
            validateRevenueContributions(contributions);
            encoder.write(static_cast<uint16_t>(contributions.size()));
            for (const auto& contribution : contributions)
            {
                encoder.write(entityValue(contribution.vehicle));
                encoder.write(contribution.weight);
            }
        }

        std::vector<RevenueContribution> decodeRevenueContributions(Decoder& decoder)
        {
            const auto count = decoder.read<uint16_t>();
            require(count <= S5::Limits::kMaxEntities && count <= decoder.remaining() / 6, "Too many CargoDist revenue contributions");
            std::vector<RevenueContribution> contributions;
            contributions.reserve(count);
            for (uint16_t i = 0; i < count; ++i)
            {
                contributions.push_back({ EntityId(decoder.read<uint16_t>()), decoder.read<uint32_t>() });
            }
            validateRevenueContributions(contributions);
            return contributions;
        }

        void encodePackets(Encoder& encoder, const PacketList& packets, uint32_t maxQuantity)
        {
            require(packets.size() <= std::numeric_limits<uint32_t>::max(), "Too many CargoDist packets");
            encoder.write(static_cast<uint32_t>(packets.size()));
            uint64_t quantity = 0;
            for (const auto& packet : packets.packets())
            {
                require(packet.quantity != 0, "CargoDist packet has zero quantity");
                require(packet.transferCredit >= 0 && packet.transferCredit <= static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * packet.quantity, "CargoDist packet has invalid transfer credit");
                require(isValidStation(packet.origin), "CargoDist packet has invalid origin");
                require(isValidStation(packet.nextHop, true), "CargoDist packet has invalid next hop");
                require(isValidStation(packet.destination, true), "CargoDist packet has invalid destination");
                validatePacketMetadata(packet);
                validateServiceLeg(packet.departure, packet.arrival, packet.nextHop == StationId::null, "Invalid CargoDist packet service points");
                quantity += packet.quantity;
                require(quantity <= maxQuantity, "CargoDist packet quantity exceeds supported capacity");
                encoder.write(packet.quantity);
                encoder.write(stationValue(packet.origin));
                encoder.write(stationValue(packet.nextHop));
                encoder.write(packet.age);
                encoder.write<uint8_t>(0);
                encodeServicePoint(encoder, packet.departure);
                encodeServicePoint(encoder, packet.arrival);
                encoder.write(stationValue(packet.destination));
                encoder.write(packet.transferCredit);
                encoder.write(static_cast<uint8_t>(packet.tripKind));
                encoder.write(enumValue(packet.holidayIndustry));
                encoder.write(enumValue(packet.homeTown));
                encoder.write<uint8_t>(0);
                encoder.write(stationValue(packet.legOrigin));
                encodeRevenueContributions(encoder, packet.revenueContributions);
            }
        }

        PacketList decodePackets(Decoder& decoder, uint32_t maxQuantity, uint16_t version)
        {
            const auto count = decoder.read<uint32_t>();
            require(count <= maxQuantity, "Too many CargoDist packets");
            const auto packetSize = version >= 11 ? 35U : version >= 10 ? 31U
                : version >= 5                                          ? 26U
                : version >= 4                                          ? 18U
                : version >= 3                                          ? 16U
                                                                        : 8U;
            require(count <= decoder.remaining() / packetSize, "Too many CargoDist packets");
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
                if (version >= 3)
                {
                    packet.departure = decodeServicePoint(decoder);
                    packet.arrival = decodeServicePoint(decoder);
                    validateServiceLeg(packet.departure, packet.arrival, packet.nextHop == StationId::null, "Invalid CargoDist packet service points");
                }
                if (version >= 4)
                {
                    packet.destination = StationId(decoder.read<uint16_t>());
                }
                if (version >= 5)
                {
                    packet.transferCredit = decoder.readInt64();
                }
                if (version >= 10)
                {
                    packet.tripKind = static_cast<PassengerTripKind>(decoder.read<uint8_t>());
                    packet.holidayIndustry = IndustryId(decoder.read<uint8_t>());
                    packet.homeTown = TownId(decoder.read<uint16_t>());
                    require(decoder.read<uint8_t>() == 0, "Invalid CargoDist packet padding");
                }
                if (version >= 11)
                {
                    packet.legOrigin = StationId(decoder.read<uint16_t>());
                    packet.revenueContributions = decodeRevenueContributions(decoder);
                }
                require(packet.quantity != 0, "CargoDist packet has zero quantity");
                require(packet.transferCredit >= 0 && packet.transferCredit <= static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * packet.quantity, "CargoDist packet has invalid transfer credit");
                require(isValidStation(packet.origin), "CargoDist packet has invalid origin");
                require(isValidStation(packet.nextHop, true), "CargoDist packet has invalid next hop");
                require(isValidStation(packet.destination, true), "CargoDist packet has invalid destination");
                validatePacketMetadata(packet);
                require(packet.quantity <= maxQuantity - quantity, "CargoDist packet quantity exceeds supported capacity");
                quantity += packet.quantity;
                packets.push_back(packet);
            }
            return PacketList::fromPackets(std::move(packets));
        }

        void validateFlowOptions(const FlowKey& key, const std::vector<FlowOption>& options, bool serviceAware)
        {
            uint64_t totalWeight = 0;
            for (const auto& option : options)
            {
                require(isValidStation(option.via) && option.weight != 0, "Invalid CargoDist flow option");
                if (serviceAware)
                {
                    validateServiceLeg(option.departure, option.arrival, option.via == key.station, "Invalid CargoDist flow service points");
                }
                totalWeight += option.weight;
            }
            require(totalWeight <= std::numeric_limits<uint32_t>::max(), "CargoDist flow weight exceeds supported range");

            const auto limit = static_cast<int64_t>(totalWeight * kFlowCursorScale);
            int64_t currentTotal = 0;
            for (size_t i = 0; i < options.size(); ++i)
            {
                const auto& option = options[i];
                require(option.current >= -limit && option.current <= limit, "Invalid CargoDist flow cursor");
                if (i != 0)
                {
                    const auto& previous = options[i - 1];
                    const auto sorted = serviceAware
                        ? std::tie(previous.via, previous.departure, previous.arrival) < std::tie(option.via, option.departure, option.arrival)
                        : stationValue(previous.via) < stationValue(option.via);
                    require(sorted, "Unsorted CargoDist flow options");
                }
                require((option.current <= 0 || currentTotal <= std::numeric_limits<int64_t>::max() - option.current) && (option.current >= 0 || currentTotal >= std::numeric_limits<int64_t>::min() - option.current), "CargoDist flow cursor total overflow");
                currentTotal += option.current;
            }
            require(currentTotal == 0, "Unbalanced CargoDist flow cursors");
        }

        void validateDestinationOptions(const std::vector<DestinationOption>& options)
        {
            uint64_t totalWeight = 0;
            for (size_t i = 0; i < options.size(); ++i)
            {
                const auto& option = options[i];
                require(isValidStation(option.destination) && option.weight != 0, "Invalid CargoDist destination option");
                if (i != 0)
                {
                    require(stationValue(options[i - 1].destination) < stationValue(option.destination), "Unsorted CargoDist destination options");
                }
                totalWeight += option.weight;
            }
            require(totalWeight <= std::numeric_limits<uint32_t>::max(), "CargoDist destination weight exceeds supported range");
            const auto limit = static_cast<int64_t>(totalWeight * kFlowCursorScale);
            int64_t currentTotal = 0;
            for (const auto& option : options)
            {
                require(option.current >= -limit && option.current <= limit, "Invalid CargoDist destination cursor");
                require((option.current <= 0 || currentTotal <= std::numeric_limits<int64_t>::max() - option.current) && (option.current >= 0 || currentTotal >= std::numeric_limits<int64_t>::min() - option.current), "CargoDist destination cursor total overflow");
                currentTotal += option.current;
            }
            require(currentTotal == 0, "Unbalanced CargoDist destination cursors");
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
            encodePackets(payload, packets, std::numeric_limits<uint32_t>::max());
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
            require(key.cargo < state.settings.modes.size() && isValidStation(key.station) && isValidStation(key.origin) && isValidStation(key.destination) && isValidServicePoint(key.incoming), "Invalid CargoDist flow key");
            require(options.size() <= std::numeric_limits<uint16_t>::max(), "Too many CargoDist flow options");
            validateFlowOptions(key, options, true);
            payload.write(key.cargo);
            payload.write<uint8_t>(0);
            payload.write(stationValue(key.station));
            payload.write(stationValue(key.origin));
            encodeServicePoint(payload, key.incoming);
            payload.write(stationValue(key.destination));
            payload.write(static_cast<uint16_t>(options.size()));
            for (const auto& option : options)
            {
                payload.write(stationValue(option.via));
                payload.write<uint16_t>(0);
                payload.write(option.weight);
                payload.write(option.current);
                encodeServicePoint(payload, option.departure);
                encodeServicePoint(payload, option.arrival);
            }
        }

        payload.write(countMatching(state.stationAttraction, [](const auto& item) { return item.second != 0; }));
        for (const auto& [key, attraction] : state.stationAttraction)
        {
            if (attraction == 0)
            {
                continue;
            }
            require(key.cargo < state.settings.modes.size() && isValidStation(key.station), "Invalid CargoDist station attraction key");
            payload.write(stationValue(key.station));
            payload.write(key.cargo);
            payload.write<uint8_t>(0);
            payload.write(attraction);
        }

        payload.write(countMatching(state.destinationFlows, [](const auto& item) { return !item.second.empty(); }));
        for (const auto& [key, options] : state.destinationFlows)
        {
            if (options.empty())
            {
                continue;
            }
            require(key.cargo < state.settings.modes.size() && isValidStation(key.station) && isValidStation(key.origin) && isValidServicePoint(key.incoming), "Invalid CargoDist destination flow key");
            require(options.size() <= std::numeric_limits<uint16_t>::max(), "Too many CargoDist destination options");
            validateDestinationOptions(options);
            payload.write(key.cargo);
            payload.write<uint8_t>(0);
            payload.write(stationValue(key.station));
            payload.write(stationValue(key.origin));
            encodeServicePoint(payload, key.incoming);
            payload.write(static_cast<uint16_t>(options.size()));
            for (const auto& option : options)
            {
                payload.write(stationValue(option.destination));
                payload.write<uint16_t>(0);
                payload.write(option.weight);
                payload.write(option.current);
            }
        }

        payload.write(countMatching(state.pendingVehicleRevenueAdjustments, [](const auto& item) { return item.second != 0; }));
        for (const auto& [vehicle, adjustment] : state.pendingVehicleRevenueAdjustments)
        {
            if (adjustment == 0)
            {
                continue;
            }
            require(entityValue(vehicle) < S5::Limits::kMaxEntities, "Invalid CargoDist pending revenue vehicle");
            payload.write(entityValue(vehicle));
            payload.write(adjustment);
        }

        require(state.hasStationAccessibilitySnapshot || state.stationAccessibility.empty(), "CargoDist station accessibility has no committed snapshot");
        require(state.stationAccessibility.size() <= S5::Limits::kMaxStations, "Too many CargoDist station accessibility entries");
        payload.write(static_cast<uint8_t>(state.hasStationAccessibilitySnapshot));
        payload.write<uint8_t>(0);
        payload.write<uint16_t>(0);
        payload.write(static_cast<uint32_t>(state.stationAccessibility.size()));
        for (const auto& [station, accessibility] : state.stationAccessibility)
        {
            require(isValidStation(station) && accessibility != 0, "Invalid CargoDist station accessibility entry");
            payload.write(stationValue(station));
            payload.write(accessibility);
        }

        require(state.resorts.size() <= S5::Limits::kMaxIndustries, "Too many CargoDist resort activity entries");
        payload.write(static_cast<uint32_t>(state.resorts.size()));
        for (const auto& [industry, activity] : state.resorts)
        {
            require(isValidIndustry(industry) && activity.popularity <= 100, "Invalid CargoDist resort activity");
            payload.write(enumValue(industry));
            payload.write(activity.popularity);
            payload.write(activity.liveSlopes);
            payload.write(activity.capacity);
            payload.write<uint16_t>(0);
            payload.write(activity.guestDays);
        }

        require(state.holidaySources.size() <= kMaxStationLists, "Too many CargoDist holiday source entries");
        payload.write(static_cast<uint32_t>(state.holidaySources.size()));
        for (const auto& [key, source] : state.holidaySources)
        {
            require(isValidStation(key.station) && key.cargo < state.settings.modes.size() && state.settings.modes[key.cargo] == DistributionMode::asymmetric && source.remainder < 100, "Invalid CargoDist holiday source");
            payload.write(stationValue(key.station));
            payload.write(key.cargo);
            payload.write(source.remainder);
            payload.write(source.sequence);
        }

        require(std::is_sorted(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end()), "Non-canonical CargoDist pending holiday returns");
        require(state.pendingHolidayReturns.size() <= kMaxFlowLists, "Too many CargoDist pending holiday returns");
        payload.write(static_cast<uint32_t>(state.pendingHolidayReturns.size()));
        for (const auto& pending : state.pendingHolidayReturns)
        {
            require(pending.quantity != 0 && pending.cargo < state.settings.modes.size() && state.settings.modes[pending.cargo] == DistributionMode::asymmetric && isValidStation(pending.resortStation, true) && isValidStation(pending.homeStation, true) && isValidTown(pending.homeTown) && isValidIndustry(pending.resort) && pending.transferCredit >= 0 && pending.transferCredit <= static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * pending.quantity && (pending.released || (pending.age == 0 && pending.transferCredit == 0 && pending.revenueContributions.empty())), "Invalid CargoDist pending holiday return");
            payload.write(pending.releaseDay);
            payload.write(pending.quantity);
            payload.write(stationValue(pending.resortStation));
            payload.write(stationValue(pending.homeStation));
            payload.write(enumValue(pending.homeTown));
            payload.write(enumValue(pending.resort));
            payload.write(pending.cargo);
            payload.write(pending.age);
            payload.write(static_cast<uint8_t>(pending.released));
            payload.write(pending.transferCredit);
            encodeRevenueContributions(payload, pending.revenueContributions);
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
        const auto version = decoder.read<uint16_t>();
        require(version >= 1 && version <= kVersion, "Unsupported CargoDist save version");
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
            const auto maximumQuantity = version >= 6 ? std::numeric_limits<uint32_t>::max() : std::numeric_limits<uint16_t>::max();
            require(state.stationCargo.emplace(key, decodePackets(decoder, maximumQuantity, version)).second, "Duplicate CargoDist station cargo key");
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
            require(state.vehicleCargo.emplace(key, decodePackets(decoder, std::numeric_limits<uint8_t>::max(), version)).second, "Duplicate CargoDist vehicle cargo key");
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
            if (version >= 3)
            {
                key.incoming = decodeServicePoint(decoder);
            }
            if (version >= 4)
            {
                key.destination = StationId(decoder.read<uint16_t>());
            }
            const auto optionCount = decoder.read<uint16_t>();
            require(key.cargo < state.settings.modes.size() && isValidStation(key.station) && isValidStation(key.origin) && isValidStation(key.destination, version < 4) && isValidServicePoint(key.incoming), "Invalid CargoDist flow key");
            require(optionCount != 0, "Invalid CargoDist flow option count");
            std::vector<FlowOption> options;
            options.reserve(optionCount);
            for (uint16_t j = 0; j < optionCount; ++j)
            {
                FlowOption option;
                option.via = StationId(decoder.read<uint16_t>());
                decoder.read<uint16_t>();
                option.weight = decoder.read<uint32_t>();
                option.current = decoder.readInt64();
                if (version >= 3)
                {
                    option.departure = decodeServicePoint(decoder);
                    option.arrival = decodeServicePoint(decoder);
                }
                options.push_back(option);
            }
            validateFlowOptions(key, options, version >= 3);
            require(state.flows.emplace(key, std::move(options)).second, "Duplicate CargoDist flow key");
        }

        if (version >= 2)
        {
            const auto attractionCount = decoder.read<uint32_t>();
            require(attractionCount <= kMaxStationLists, "Too many CargoDist station attraction entries");
            for (uint32_t i = 0; i < attractionCount; ++i)
            {
                StationCargoKey key;
                key.station = StationId(decoder.read<uint16_t>());
                key.cargo = decoder.read<uint8_t>();
                decoder.read<uint8_t>();
                const auto attraction = decoder.read<uint32_t>();
                require(isValidStation(key.station) && key.cargo < state.settings.modes.size() && attraction != 0, "Invalid CargoDist station attraction entry");
                require(state.stationAttraction.emplace(key, attraction).second, "Duplicate CargoDist station attraction key");
            }
        }

        if (version >= 4)
        {
            const auto destinationFlowCount = decoder.read<uint32_t>();
            require(destinationFlowCount <= kMaxFlowLists, "Too many CargoDist destination flow entries");
            for (uint32_t i = 0; i < destinationFlowCount; ++i)
            {
                DestinationFlowKey key;
                key.cargo = decoder.read<uint8_t>();
                decoder.read<uint8_t>();
                key.station = StationId(decoder.read<uint16_t>());
                key.origin = StationId(decoder.read<uint16_t>());
                key.incoming = decodeServicePoint(decoder);
                const auto optionCount = decoder.read<uint16_t>();
                require(key.cargo < state.settings.modes.size() && isValidStation(key.station) && isValidStation(key.origin) && isValidServicePoint(key.incoming), "Invalid CargoDist destination flow key");
                require(optionCount != 0, "Invalid CargoDist destination option count");
                std::vector<DestinationOption> options;
                options.reserve(optionCount);
                for (uint16_t j = 0; j < optionCount; ++j)
                {
                    DestinationOption option;
                    option.destination = StationId(decoder.read<uint16_t>());
                    decoder.read<uint16_t>();
                    option.weight = decoder.read<uint32_t>();
                    option.current = decoder.readInt64();
                    options.push_back(option);
                }
                validateDestinationOptions(options);
                require(state.destinationFlows.emplace(key, std::move(options)).second, "Duplicate CargoDist destination flow key");
            }
        }

        if (version >= 5)
        {
            const auto pendingAdjustmentCount = decoder.read<uint32_t>();
            require(pendingAdjustmentCount <= S5::Limits::kMaxEntities, "Too many CargoDist pending revenue adjustments");
            for (uint32_t i = 0; i < pendingAdjustmentCount; ++i)
            {
                const auto vehicle = EntityId(decoder.read<uint16_t>());
                const auto adjustment = decoder.readInt64();
                require(entityValue(vehicle) < S5::Limits::kMaxEntities && adjustment != 0, "Invalid CargoDist pending revenue vehicle");
                require(state.pendingVehicleRevenueAdjustments.emplace(vehicle, adjustment).second, "Duplicate CargoDist pending revenue vehicle");
            }
        }

        if (version >= 8)
        {
            const auto hasStationAccessibilitySnapshot = decoder.read<uint8_t>();
            require(hasStationAccessibilitySnapshot <= 1, "Invalid CargoDist station accessibility snapshot state");
            state.hasStationAccessibilitySnapshot = hasStationAccessibilitySnapshot != 0;
            decoder.read<uint8_t>();
            decoder.read<uint16_t>();
            const auto accessibilityCount = decoder.read<uint32_t>();
            require(accessibilityCount <= S5::Limits::kMaxStations && accessibilityCount <= decoder.remaining() / (sizeof(uint16_t) + sizeof(uint32_t)), "Too many CargoDist station accessibility entries");
            require(state.hasStationAccessibilitySnapshot || accessibilityCount == 0, "CargoDist station accessibility has no committed snapshot");
            uint16_t previousStation{};
            for (uint32_t i = 0; i < accessibilityCount; ++i)
            {
                const auto station = StationId(decoder.read<uint16_t>());
                const auto accessibility = decoder.read<uint32_t>();
                require(isValidStation(station) && accessibility != 0, "Invalid CargoDist station accessibility entry");
                require(i == 0 || previousStation < stationValue(station), "Non-canonical CargoDist station accessibility order");
                state.stationAccessibility.emplace(station, accessibility);
                previousStation = stationValue(station);
            }
        }

        if (version >= 10)
        {
            const auto resortCount = decoder.read<uint32_t>();
            require(resortCount <= S5::Limits::kMaxIndustries && resortCount <= decoder.remaining() / 12, "Too many CargoDist resort activity entries");
            uint8_t previousIndustry{};
            for (uint32_t i = 0; i < resortCount; ++i)
            {
                const auto industry = IndustryId(decoder.read<uint8_t>());
                ResortActivity activity;
                activity.popularity = decoder.read<uint8_t>();
                activity.liveSlopes = decoder.read<uint16_t>();
                activity.capacity = decoder.read<uint16_t>();
                require(decoder.read<uint16_t>() == 0, "Invalid CargoDist resort activity padding");
                activity.guestDays = decoder.read<uint32_t>();
                require(isValidIndustry(industry) && activity.popularity <= 100 && (i == 0 || previousIndustry < enumValue(industry)) && state.resorts.emplace(industry, activity).second, "Invalid CargoDist resort activity");
                previousIndustry = enumValue(industry);
            }

            const auto sourceCount = decoder.read<uint32_t>();
            require(sourceCount <= kMaxStationLists && sourceCount <= decoder.remaining() / 8, "Too many CargoDist holiday source entries");
            std::optional<StationCargoKey> previousSource;
            for (uint32_t i = 0; i < sourceCount; ++i)
            {
                StationCargoKey key;
                key.station = StationId(decoder.read<uint16_t>());
                key.cargo = decoder.read<uint8_t>();
                HolidaySourceState source;
                source.remainder = decoder.read<uint8_t>();
                source.sequence = decoder.read<uint32_t>();
                require(isValidStation(key.station) && key.cargo < state.settings.modes.size() && state.settings.modes[key.cargo] == DistributionMode::asymmetric && source.remainder < 100 && (!previousSource.has_value() || *previousSource < key) && state.holidaySources.emplace(key, source).second, "Invalid CargoDist holiday source");
                previousSource = key;
            }

            const auto pendingCount = decoder.read<uint32_t>();
            require(pendingCount <= kMaxFlowLists && pendingCount <= decoder.remaining() / (version >= 11 ? 26U : 24U), "Too many CargoDist pending holiday returns");
            for (uint32_t i = 0; i < pendingCount; ++i)
            {
                PendingHolidayReturn pending;
                pending.releaseDay = decoder.read<uint32_t>();
                pending.quantity = decoder.read<uint16_t>();
                pending.resortStation = StationId(decoder.read<uint16_t>());
                pending.homeStation = StationId(decoder.read<uint16_t>());
                pending.homeTown = TownId(decoder.read<uint16_t>());
                pending.resort = IndustryId(decoder.read<uint8_t>());
                pending.cargo = decoder.read<uint8_t>();
                pending.age = decoder.read<uint8_t>();
                const auto released = decoder.read<uint8_t>();
                require(released <= 1, "Invalid CargoDist pending holiday return state");
                pending.released = released != 0;
                pending.transferCredit = decoder.readInt64();
                if (version >= 11)
                {
                    pending.revenueContributions = decodeRevenueContributions(decoder);
                }
                require(pending.quantity != 0 && pending.cargo < state.settings.modes.size() && state.settings.modes[pending.cargo] == DistributionMode::asymmetric && isValidStation(pending.resortStation, true) && isValidStation(pending.homeStation, true) && isValidTown(pending.homeTown) && isValidIndustry(pending.resort) && pending.transferCredit >= 0 && pending.transferCredit <= static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * pending.quantity && (pending.released || (pending.age == 0 && pending.transferCredit == 0 && pending.revenueContributions.empty())), "Invalid CargoDist pending holiday return");
                state.pendingHolidayReturns.push_back(pending);
            }
            require(std::is_sorted(state.pendingHolidayReturns.begin(), state.pendingHolidayReturns.end()), "Non-canonical CargoDist pending holiday returns");
        }

        require(decoder.empty(), "Trailing CargoDist save data");
        if (version < 4)
        {
            state.flows.clear();
            state.destinationFlows.clear();
        }
        if (version < 7)
        {
            state.stationAttraction.clear();
            state.graphDirty = true;
        }
        if (version < 8)
        {
            state.stationAccessibility.clear();
            state.hasStationAccessibilitySnapshot = false;
            state.requiresStationMetadataRefresh = true;
        }
        if (version < 9)
        {
            state.graphDirty = true;
        }
        return state;
    }
}
