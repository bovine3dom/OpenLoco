#include "Ui/CargoRouteTree.h"

#include "Graphics/Colour.h"
#include "Graphics/Gfx.h"
#include "Graphics/RenderTarget.h"
#include "Graphics/TextRenderer.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/Formatting.h"
#include "Localisation/StringIds.h"
#include "Ui/WindowManager.h"
#include "World/StationManager.h"
#include <OpenLoco/Utility/String.hpp>
#include <algorithm>
#include <limits>

namespace OpenLoco::Ui::CargoRouteTree
{
    namespace
    {
        using RouteOrder = std::array<CargoDist::CargoRouteField, 3>;

        constexpr std::array<RouteOrder, 6> kGroupOrders = { {
            { CargoDist::CargoRouteField::origin, CargoDist::CargoRouteField::nextHop, CargoDist::CargoRouteField::destination },
            { CargoDist::CargoRouteField::origin, CargoDist::CargoRouteField::destination, CargoDist::CargoRouteField::nextHop },
            { CargoDist::CargoRouteField::nextHop, CargoDist::CargoRouteField::origin, CargoDist::CargoRouteField::destination },
            { CargoDist::CargoRouteField::nextHop, CargoDist::CargoRouteField::destination, CargoDist::CargoRouteField::origin },
            { CargoDist::CargoRouteField::destination, CargoDist::CargoRouteField::origin, CargoDist::CargoRouteField::nextHop },
            { CargoDist::CargoRouteField::destination, CargoDist::CargoRouteField::nextHop, CargoDist::CargoRouteField::origin },
        } };

        constexpr std::array<StringId, 6> kGroupOrderNames = {
            StringIds::cargo_group_source_via_destination,
            StringIds::cargo_group_source_destination_via,
            StringIds::cargo_group_via_source_destination,
            StringIds::cargo_group_via_destination_source,
            StringIds::cargo_group_destination_source_via,
            StringIds::cargo_group_destination_via_source,
        };

        constexpr std::array<StringId, 2> kSortModeNames = {
            StringIds::cargo_sort_station,
            StringIds::cargo_sort_amount_waiting,
        };

        bool isValidStation(const StationId stationId)
        {
            const auto* station = StationManager::get(stationId);
            return station != nullptr && !station->empty();
        }

        bool stationLess(const StationId lhs, const StationId rhs)
        {
            if (lhs == rhs)
            {
                return false;
            }

            const auto lhsValid = isValidStation(lhs);
            const auto rhsValid = isValidStation(rhs);
            if (lhsValid != rhsValid)
            {
                return lhsValid;
            }
            if (!lhsValid)
            {
                return enumValue(lhs) < enumValue(rhs);
            }

            std::array<char, 256> lhsName{};
            {
                const auto* station = StationManager::get(lhs);
                FormatArguments args{};
                args.push(station->town);
                StringManager::formatString(lhsName.data(), lhsName.size(), station->name, args);
            }
            std::array<char, 256> rhsName{};
            {
                const auto* station = StationManager::get(rhs);
                FormatArguments args{};
                args.push(station->town);
                StringManager::formatString(rhsName.data(), rhsName.size(), station->name, args);
            }
            const auto comparison = Utility::strlogicalcmp(lhsName.data(), rhsName.data());
            return comparison == 0 ? enumValue(lhs) < enumValue(rhs) : comparison < 0;
        }

        void appendStationArguments(FormatArguments& args, const StationId stationId)
        {
            const auto* station = StationManager::get(stationId);
            args.push(station->name);
            args.push(station->town);
        }

        StringId getFormat(const Row& row, FormatArguments& args)
        {
            args.push<int32_t>(static_cast<int32_t>(std::min<uint64_t>(row.quantity, std::numeric_limits<int32_t>::max())));
            const auto hasStation = isValidStation(row.station);
            if (hasStation)
            {
                appendStationArguments(args, row.station);
            }

            switch (row.field)
            {
                case CargoDist::CargoRouteField::origin:
                    return hasStation ? StringIds::station_cargo_group_source : StringIds::station_cargo_group_source_unknown;
                case CargoDist::CargoRouteField::destination:
                    return hasStation ? StringIds::station_cargo_group_destination : StringIds::station_cargo_group_destination_pending;
                case CargoDist::CargoRouteField::nextHop:
                    return hasStation ? StringIds::station_cargo_group_via : StringIds::station_cargo_group_awaiting_route;
            }
            return StringIds::empty;
        }

        int16_t getDisclosureLeft(const Row& row, const int16_t xOffset)
        {
            return static_cast<int16_t>(xOffset + 2 + row.depth * 8);
        }

        int16_t getTextLeft(const Row& row, const int16_t xOffset)
        {
            return static_cast<int16_t>(xOffset + 10 + row.depth * 8);
        }

        uint16_t getTextWidth(const Row& row)
        {
            std::array<char, 512> buffer{};
            FormatArguments args{};
            StringManager::formatString(buffer.data(), buffer.size(), getFormat(row, args), args);
            return Gfx::TextRenderer::getStringWidth(Gfx::Font::medium_bold, buffer.data());
        }

        void appendRows(std::vector<Row>& rows, const std::vector<CargoDist::CargoRouteNode>& nodes, const RouteOrder& order, const std::set<GroupKey>& expandedGroups, const uint8_t depth, const GroupKey& parentKey, const size_t maxRows, size_t& omittedRows)
        {
            if (depth >= order.size())
            {
                return;
            }
            for (const auto& node : nodes)
            {
                auto key = parentKey;
                key.depth = depth + 1;
                key.stations[depth] = node.station;
                const auto expandable = !node.children.empty();
                const auto expanded = expandable && expandedGroups.contains(key);
                if (rows.size() < maxRows)
                {
                    rows.push_back({
                        .depth = static_cast<uint8_t>(depth + 1),
                        .expandable = expandable,
                        .expanded = expanded,
                        .field = order[depth],
                        .station = node.station,
                        .quantity = node.quantity,
                        .key = key,
                    });
                }
                else
                {
                    ++omittedRows;
                }

                if (expanded)
                {
                    appendRows(rows, node.children, order, expandedGroups, depth + 1, key, maxRows, omittedRows);
                }
            }
        }
    }

    const std::array<CargoDist::CargoRouteField, 3>& getOrder(const GroupOrder order)
    {
        return kGroupOrders[static_cast<size_t>(order)];
    }

    std::span<const StringId> getGroupOrderNames()
    {
        return kGroupOrderNames;
    }

    std::span<const StringId> getSortModeNames()
    {
        return kSortModeNames;
    }

    void sortTree(std::vector<CargoDist::CargoRouteNode>& nodes, const SortMode sortMode)
    {
        std::sort(nodes.begin(), nodes.end(), [sortMode](const auto& lhs, const auto& rhs) {
            if (sortMode == SortMode::amountWaiting)
            {
                return lhs.quantity == rhs.quantity
                    ? enumValue(lhs.station) < enumValue(rhs.station)
                    : lhs.quantity > rhs.quantity;
            }
            return stationLess(lhs.station, rhs.station);
        });
        for (auto& node : nodes)
        {
            sortTree(node.children, sortMode);
        }
    }

    void expandAllGroups(std::set<GroupKey>& expandedGroups, const std::vector<CargoDist::CargoRouteNode>& nodes)
    {
        const auto addGroups = [&](const auto& self, const auto& children, GroupKey parent, const uint8_t depth) -> void {
            for (const auto& node : children)
            {
                auto key = parent;
                key.depth = depth + 1;
                key.stations[depth] = node.station;
                if (node.children.empty())
                {
                    continue;
                }
                expandedGroups.insert(key);
                self(self, node.children, key, depth + 1);
            }
        };
        addGroups(addGroups, nodes, {}, 0);
    }

    void appendRows(std::vector<Row>& rows, const std::vector<CargoDist::CargoRouteNode>& nodes, const GroupOrder order, const std::set<GroupKey>& expandedGroups, const size_t maxRows, size_t& omittedRows)
    {
        appendRows(rows, nodes, getOrder(order), expandedGroups, 0, {}, maxRows, omittedRows);
    }

    void drawDisclosure(Gfx::DrawingContext& drawingCtx, const int16_t x, const int16_t y, const bool expanded)
    {
        drawingCtx.fillRect(x, y, x + 4, y, PaletteIndex::black0, Gfx::RectFlags::none);
        if (!expanded)
        {
            drawingCtx.fillRect(x + 2, y - 2, x + 2, y + 2, PaletteIndex::black0, Gfx::RectFlags::none);
        }
    }

    void drawRow(Gfx::DrawingContext& drawingCtx, const Row& row, const int32_t y, const int32_t width, const int16_t xOffset)
    {
        if (row.expandable)
        {
            drawDisclosure(drawingCtx, getDisclosureLeft(row, xOffset), static_cast<int16_t>(y + 5), row.expanded);
        }

        auto tr = Gfx::TextRenderer(drawingCtx);
        FormatArguments args{};
        const auto format = getFormat(row, args);
        const auto textLeft = getTextLeft(row, xOffset);
        tr.drawStringLeftClipped({ textLeft, static_cast<int16_t>(y + 1) }, std::max(width - textLeft - 14, 0), Colour::black, format, args);
    }

    bool isDisclosureHit(const Row& row, const int16_t x, const int16_t xOffset)
    {
        const auto left = getDisclosureLeft(row, xOffset);
        return row.expandable && x >= left && x <= left + 6;
    }

    bool isStationLinkHit(const Row& row, const int16_t x, const int32_t width, const int16_t xOffset)
    {
        if (!isValidStation(row.station))
        {
            return false;
        }

        const auto left = getTextLeft(row, xOffset);
        const auto right = std::min<int32_t>(left + getTextWidth(row), width - 14);
        return x >= left && x < right;
    }

    void centreOnStation(const StationId stationId)
    {
        const auto* station = StationManager::get(stationId);
        if (station == nullptr || station->empty())
        {
            return;
        }

        auto* main = WindowManager::getMainWindow();
        if (main != nullptr)
        {
            Ui::Windows::Main::viewportFocusOnEntity(*main, EntityId::null);
            main->viewportCentreOnTile({ station->x, station->y, static_cast<coord_t>(station->z + 32) });
        }
    }
}
