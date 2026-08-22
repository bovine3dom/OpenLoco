#pragma once

#include <OpenLoco/CargoDist/CargoDist.h>
#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <set>
#include <span>
#include <vector>

namespace OpenLoco::Gfx
{
    class DrawingContext;
}

namespace OpenLoco::Ui::CargoRouteTree
{
    enum class GroupOrder : uint8_t
    {
        sourceViaDestination,
        sourceDestinationVia,
        viaSourceDestination,
        viaDestinationSource,
        destinationSourceVia,
        destinationViaSource,
    };

    enum class SortMode : uint8_t
    {
        station,
        amountWaiting,
    };

    struct GroupKey
    {
        uint8_t depth{};
        std::array<StationId, 3> stations{ StationId::null, StationId::null, StationId::null };

        auto operator<=>(const GroupKey&) const = default;
    };

    struct Row
    {
        uint8_t depth{};
        bool expandable{};
        bool expanded{};
        CargoDist::CargoRouteField field{};
        StationId station = StationId::null;
        uint64_t quantity{};
        GroupKey key{};
    };

    inline constexpr int32_t kRowHeight = 10;

    const std::array<CargoDist::CargoRouteField, 3>& getOrder(GroupOrder order);
    std::span<const StringId> getGroupOrderNames();
    std::span<const StringId> getSortModeNames();
    void sortTree(std::vector<CargoDist::CargoRouteNode>& nodes, SortMode sortMode);
    void appendRows(std::vector<Row>& rows, const std::vector<CargoDist::CargoRouteNode>& nodes, GroupOrder order, const std::set<GroupKey>& expandedGroups, size_t maxRows, size_t& omittedRows);
    void drawDisclosure(Gfx::DrawingContext& drawingCtx, int16_t x, int16_t y, bool expanded);
    void drawRow(Gfx::DrawingContext& drawingCtx, const Row& row, int32_t y, int32_t width, int16_t xOffset = 0);
    bool isDisclosureHit(const Row& row, int16_t x, int16_t xOffset = 0);
    bool isStationLinkHit(const Row& row, int16_t x, int32_t width, int16_t xOffset = 0);
    void centreOnStation(StationId stationId);
}
