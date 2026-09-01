// SPDX-License-Identifier: MIT
#include "Ui/Windows/CargoFlowOverlay.h"

#include "CargoDist/CargoDist.h"
#include "CargoDist/FlowAnalytics.h"
#include "Date.h"
#include "GameState.h"
#include "Graphics/DrawingContext.h"
#include "Graphics/ImageIds.h"
#include "Graphics/RenderTarget.h"
#include "Graphics/TextRenderer.h"
#include "Input.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/StringIds.h"
#include "Map/Tile.h"
#include "Map/TileManager.h"
#include "Objects/CargoObject.h"
#include "Objects/InterfaceSkinObject.h"
#include "Objects/ObjectManager.h"
#include "Ui/Dropdown.h"
#include "Ui/Widgets/CaptionWidget.h"
#include "Ui/Widgets/DropdownWidget.h"
#include "Ui/Widgets/FrameWidget.h"
#include "Ui/Widgets/ImageButtonWidget.h"
#include "Ui/Widgets/LabelWidget.h"
#include "Ui/Widgets/PanelWidget.h"
#include "Ui/WindowManager.h"
#include "Viewport.hpp"
#include "World/StationManager.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace OpenLoco::Ui::Windows::CargoFlowOverlay
{
    namespace
    {
        constexpr uint8_t kNoCargo = 0xFF;
        constexpr Ui::Size kWindowSize = { 260, 126 };
        constexpr int16_t kLegendLeft = 96;
        constexpr int16_t kLegendTop = 95;
        constexpr int16_t kLegendCellWidth = 13;

        enum class ViewMode : uint8_t
        {
            serviceThroughput,
            servicePlan,
            destinationDemand,
            destinationGaps,
            liveCommitments,
        };

        constexpr std::array<ViewMode, 5> kViewModes = {
            ViewMode::serviceThroughput,
            ViewMode::servicePlan,
            ViewMode::destinationDemand,
            ViewMode::destinationGaps,
            ViewMode::liveCommitments,
        };
        constexpr std::array<uint16_t, 3> kHorizons = { 30, 90, 365 };

        constexpr std::array<PaletteIndex_t, 12> kSaturationColours = {
            PaletteIndex::black5,
            PaletteIndex::green5,
            PaletteIndex::green7,
            PaletteIndex::green9,
            PaletteIndex::mutedAvocadoGreen9,
            PaletteIndex::yellow8,
            PaletteIndex::amber8,
            PaletteIndex::orangeA,
            PaletteIndex::orange8,
            PaletteIndex::redA,
            PaletteIndex::red8,
            PaletteIndex::red6,
        };

        struct DisplayLink
        {
            StationId from = StationId::null;
            StationId to = StationId::null;
            uint64_t demand{};
            std::optional<uint64_t> capacity;
            uint64_t plannedDemand{};
            uint64_t committedDemand{};
            uint64_t waitingDemand{};
            uint64_t incomingDemand{};
            uint8_t scaleBucket{};
            std::optional<CargoDist::FlowAnalytics::Endpoint> originEndpoint;
            std::optional<CargoDist::FlowAnalytics::Endpoint> destinationEndpoint;
            CargoDist::FlowAnalytics::GapReason gap{};
        };

        struct Snapshot
        {
            uint8_t cargo = kNoCargo;
            ViewMode view = ViewMode::serviceThroughput;
            uint16_t horizonDays = 90;
            uint64_t routingRevision{};
            uint64_t cargoRevision{};
            uint32_t analyticsRevision{};
            uint32_t mapRevision{};
            uint32_t currentDay{};
            std::vector<DisplayLink> links;
            std::vector<CargoDist::FlowAnalytics::Endpoint> endpoints;
        };

        enum widx
        {
            frame,
            caption,
            closeButton,
            panel,
            cargoLabel,
            cargo,
            cargoButton,
            viewLabel,
            view,
            viewButton,
            periodLabel,
            period,
            periodButton,
            scaleLabel,
            scale,
            scaleButton,
            saturationLabel,
        };

        namespace Widx
        {
            constexpr WidgetId kFrame{ "frame" };
            constexpr WidgetId kCaption{ "caption" };
            constexpr WidgetId kCloseButton{ "close_button" };
            constexpr WidgetId kPanel{ "panel" };
            constexpr WidgetId kCargo{ "cargo" };
            constexpr WidgetId kCargoButton{ "cargo_button" };
            constexpr WidgetId kView{ "view" };
            constexpr WidgetId kViewButton{ "view_button" };
            constexpr WidgetId kPeriod{ "period" };
            constexpr WidgetId kPeriodButton{ "period_button" };
            constexpr WidgetId kScale{ "scale" };
            constexpr WidgetId kScaleButton{ "scale_button" };
        }

        static constexpr auto kWidgets = makeWidgets(
            Widgets::Frame(Widx::kFrame, { 0, 0 }, kWindowSize, WindowColour::primary),
            Widgets::Caption(Widx::kCaption, { 1, 1 }, { kWindowSize.width - 2, 13 }, Widgets::Caption::Style::whiteText, WindowColour::primary, StringIds::cargo_flow_overlay),
            Widgets::ImageButton(Widx::kCloseButton, { kWindowSize.width - 15, 2 }, { 13, 13 }, WindowColour::primary, ImageIds::close_button, StringIds::tooltip_close_window),
            Widgets::Panel(Widx::kPanel, { 0, 15 }, { kWindowSize.width, kWindowSize.height - 15 }, WindowColour::secondary),
            Widgets::Label({ 6, 22 }, { 44, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::cargo_flow_cargo),
            Widgets::dropdownWidgets(Widx::kCargo, Widx::kCargoButton, { 50, 21 }, { 202, 12 }, WindowColour::secondary, StringIds::stringid, StringIds::tooltip_select_cargo_type),
            Widgets::Label({ 6, 40 }, { 44, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::cargo_flow_view),
            Widgets::dropdownWidgets(Widx::kView, Widx::kViewButton, { 50, 39 }, { 202, 12 }, WindowColour::secondary, StringIds::stringid),
            Widgets::Label({ 6, 58 }, { 44, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::cargo_flow_period),
            Widgets::dropdownWidgets(Widx::kPeriod, Widx::kPeriodButton, { 50, 57 }, { 202, 12 }, WindowColour::secondary, StringIds::stringid),
            Widgets::Label({ 6, 76 }, { 44, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::cargo_flow_scale),
            Widgets::dropdownWidgets(Widx::kScale, Widx::kScaleButton, { 50, 75 }, { 202, 12 }, WindowColour::secondary, StringIds::stringid),
            Widgets::Label({ 6, 94 }, { 88, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::cargo_flow_throughput_saturation));

        uint8_t _selectedCargo = kNoCargo;
        ViewMode _selectedView = ViewMode::serviceThroughput;
        uint16_t _selectedHorizon = 90;
        ScaleMode _selectedScale = ScaleMode::absolute;
        std::optional<Snapshot> _snapshot;
        std::vector<uint8_t> _dropdownCargoIds;

        std::vector<uint8_t> getEnabledCargos()
        {
            std::vector<uint8_t> cargos;
            for (uint16_t cargo = 0; cargo < ObjectManager::getMaxObjects(ObjectType::cargo); ++cargo)
            {
                if (ObjectManager::get<CargoObject>(cargo) != nullptr && CargoDist::isEnabled(cargo))
                {
                    cargos.push_back(cargo);
                }
            }
            return cargos;
        }

        bool selectValidCargo()
        {
            const auto cargos = getEnabledCargos();
            if (cargos.empty())
            {
                _selectedCargo = kNoCargo;
                return false;
            }
            if (std::find(cargos.begin(), cargos.end(), _selectedCargo) == cargos.end())
            {
                _selectedCargo = cargos.front();
            }
            return true;
        }

        StringId getViewString(const ViewMode view)
        {
            switch (view)
            {
                case ViewMode::serviceThroughput:
                    return StringIds::cargo_flow_service_throughput;
                case ViewMode::servicePlan:
                    return StringIds::cargo_flow_service_plan;
                case ViewMode::destinationDemand:
                    return StringIds::cargo_flow_destination_demand;
                case ViewMode::destinationGaps:
                    return StringIds::cargo_flow_destination_gaps;
                case ViewMode::liveCommitments:
                    return StringIds::cargo_flow_live_commitments;
            }
            return StringIds::cargo_flow_service_throughput;
        }

        StringId getPeriodString(const uint16_t horizon)
        {
            switch (horizon)
            {
                case 30:
                    return StringIds::cargo_flow_30_days;
                case 365:
                    return StringIds::cargo_flow_365_days;
                default:
                    return StringIds::cargo_flow_90_days;
            }
        }

        uint64_t calculateUtilisationBasisPoints(const uint64_t demand, const uint64_t capacity)
        {
            constexpr uint64_t kOneHundredPercent = 10'000;
            return static_cast<uint64_t>(std::min<long double>(kOneHundredPercent, static_cast<long double>(demand) * kOneHundredPercent / capacity));
        }

        void applyScale(Snapshot& snapshot, const ScaleMode scale)
        {
            std::vector<uint64_t> values;
            values.reserve(snapshot.links.size());
            for (const auto& link : snapshot.links)
            {
                values.push_back(snapshot.view != ViewMode::serviceThroughput ? link.demand : link.capacity.has_value() ? calculateUtilisationBasisPoints(link.demand, *link.capacity)
                                                                                                                        : 0);
            }
            const auto fixedMaximum = snapshot.view == ViewMode::serviceThroughput ? std::optional<uint64_t>{ 10'000 } : std::nullopt;
            const auto buckets = calculateScaleBuckets(values, scale, fixedMaximum);
            for (size_t i = 0; i < snapshot.links.size(); ++i)
            {
                snapshot.links[i].scaleBucket = buckets[i];
            }
        }

        bool refreshSnapshot()
        {
            if (!selectValidCargo())
            {
                const auto changed = _snapshot.has_value();
                _snapshot.reset();
                return changed;
            }

            const auto& state = CargoDist::getStateConst();
            const auto isLive = _selectedView == ViewMode::liveCommitments;
            const auto isDestination = _selectedView == ViewMode::destinationDemand || _selectedView == ViewMode::destinationGaps;
            if ((isLive || isDestination) && state.servicesDirty)
            {
                const auto changed = _snapshot.has_value();
                _snapshot.reset();
                return changed;
            }

            const auto analyticsRevision = CargoDist::FlowAnalytics::getRevision();
            const auto mapRevision = World::TileManager::getMapRevision();
            const auto currentDay = getCurrentDay();
            if (_snapshot.has_value() && _snapshot->cargo == _selectedCargo && _snapshot->view == _selectedView
                && _snapshot->horizonDays == _selectedHorizon
                && ((isLive && _snapshot->routingRevision == state.routingRevision && _snapshot->cargoRevision == state.cargoRevision)
                    || (isDestination && _snapshot->routingRevision == state.routingRevision && _snapshot->mapRevision == mapRevision && _snapshot->currentDay == currentDay)
                    || (!isLive && !isDestination && _snapshot->analyticsRevision == analyticsRevision)))
            {
                return false;
            }

            Snapshot snapshot{};
            snapshot.cargo = _selectedCargo;
            snapshot.view = _selectedView;
            snapshot.horizonDays = _selectedHorizon;
            snapshot.routingRevision = state.routingRevision;
            snapshot.cargoRevision = state.cargoRevision;
            snapshot.analyticsRevision = analyticsRevision;
            snapshot.mapRevision = mapRevision;
            snapshot.currentDay = currentDay;
            if (isLive)
            {
                for (const auto& edge : CargoDist::getPlannedServiceEdges(_selectedCargo))
                {
                    snapshot.links.push_back(DisplayLink{
                        .from = edge.from,
                        .to = edge.to,
                        .demand = edge.committedDemand,
                        .capacity = edge.serviceCapacity.has_value() ? std::optional<uint64_t>{ *edge.serviceCapacity } : std::nullopt,
                        .plannedDemand = edge.servicePlannedDemand,
                        .committedDemand = edge.committedDemand,
                        .waitingDemand = edge.waitingDemand,
                        .incomingDemand = edge.incomingDemand,
                        .originEndpoint = {},
                        .destinationEndpoint = {},
                    });
                }
            }
            else if (!isDestination)
            {
                for (const auto& edge : CargoDist::FlowAnalytics::getServiceSummaries(_selectedCargo, _selectedHorizon))
                {
                    if (_selectedView == ViewMode::serviceThroughput && edge.offeredCapacity == 0)
                    {
                        continue;
                    }
                    const auto capacity = _selectedView == ViewMode::serviceThroughput
                        ? edge.offeredCapacity
                        : CargoDist::FlowAnalytics::roundCapacity(edge.capacityQ16);
                    snapshot.links.push_back(DisplayLink{
                        .from = edge.from,
                        .to = edge.to,
                        .demand = _selectedView == ViewMode::serviceThroughput ? edge.observedThroughput : edge.plannedDemand,
                        .capacity = capacity == 0 ? std::nullopt : std::optional<uint64_t>{ capacity },
                        .originEndpoint = {},
                        .destinationEndpoint = {},
                    });
                }
            }
            else
            {
                const auto model = CargoDist::FlowAnalytics::getDestinationModel(_selectedCargo, _selectedHorizon);
                snapshot.endpoints = model.endpoints;
                std::map<CargoDist::FlowAnalytics::EndpointKey, CargoDist::FlowAnalytics::Endpoint> endpoints;
                for (const auto& endpoint : model.endpoints)
                {
                    endpoints.emplace(endpoint.key, endpoint);
                }
                for (const auto& flow : model.flows)
                {
                    if (_selectedView == ViewMode::destinationGaps && flow.gap == CargoDist::FlowAnalytics::GapReason::served)
                    {
                        continue;
                    }
                    const auto origin = endpoints.find(flow.origin);
                    const auto destination = endpoints.find(flow.destination);
                    if (origin == endpoints.end() || destination == endpoints.end())
                    {
                        continue;
                    }
                    snapshot.links.push_back(DisplayLink{
                        .demand = flow.demand,
                        .capacity = flow.capacity == 0 ? std::nullopt : std::optional<uint64_t>{ flow.capacity },
                        .originEndpoint = origin->second,
                        .destinationEndpoint = destination->second,
                        .gap = flow.gap,
                    });
                }
                std::ranges::sort(snapshot.links, [](const auto& lhs, const auto& rhs) {
                    if (_selectedView == ViewMode::destinationGaps)
                    {
                        const auto lhsGap = lhs.demand - std::min(lhs.demand, lhs.capacity.value_or(0));
                        const auto rhsGap = rhs.demand - std::min(rhs.demand, rhs.capacity.value_or(0));
                        if (lhsGap != rhsGap)
                        {
                            return lhsGap > rhsGap;
                        }
                    }
                    return lhs.demand > rhs.demand;
                });
                constexpr size_t kMaximumDestinationLinks = 128;
                if (snapshot.links.size() > kMaximumDestinationLinks)
                {
                    snapshot.links.resize(kMaximumDestinationLinks);
                }
            }
            applyScale(snapshot, _selectedScale);
            _snapshot = std::move(snapshot);
            return true;
        }

        void invalidateMainViewport()
        {
            if (auto* main = WindowManager::getMainWindow(); main != nullptr)
            {
                main->invalidate();
            }
        }

        bool isMainViewport(const Viewport& viewport)
        {
            const auto* main = WindowManager::getMainWindow();
            return main != nullptr && main->viewports[0] == &viewport;
        }

        bool clipLine(Point& from, Point& to, const Rect& rect)
        {
            if (rect.width() <= 0 || rect.height() <= 0)
            {
                return false;
            }

            const auto dx = static_cast<double>(to.x - from.x);
            const auto dy = static_cast<double>(to.y - from.y);
            auto start = 0.0;
            auto end = 1.0;
            const auto update = [&](double direction, double distance) {
                if (direction == 0)
                {
                    return distance >= 0;
                }
                const auto ratio = distance / direction;
                if (direction < 0)
                {
                    start = std::max(start, ratio);
                }
                else
                {
                    end = std::min(end, ratio);
                }
                return start <= end;
            };
            if (!update(-dx, from.x - rect.left()) || !update(dx, rect.right() - 1 - from.x)
                || !update(-dy, from.y - rect.top()) || !update(dy, rect.bottom() - 1 - from.y))
            {
                return false;
            }

            const auto original = from;
            from = { static_cast<int32_t>(std::lround(original.x + start * dx)), static_cast<int32_t>(std::lround(original.y + start * dy)) };
            to = { static_cast<int32_t>(std::lround(original.x + end * dx)), static_cast<int32_t>(std::lround(original.y + end * dy)) };
            return true;
        }

        Point projectPosition(const Viewport& viewport, const World::Pos3 position, const bool windowCoordinates)
        {
            const auto projected = World::gameToScreen(position, viewport.getRotation());
            if (windowCoordinates)
            {
                const auto viewOrigin = viewport.getViewOriginInRaster();
                const auto rasterPoint = Point{
                    viewport.zoom.applyInversedTo(projected.x),
                    viewport.zoom.applyInversedTo(projected.y),
                };
                return viewport.rasterToUiNearest(rasterPoint - viewOrigin) + Point{ viewport.x, viewport.y };
            }
            return {
                viewport.zoom.applyInversedTo(projected.x),
                viewport.zoom.applyInversedTo(projected.y),
            };
        }

        Point projectStation(const Viewport& viewport, const ::OpenLoco::Station& station, const bool windowCoordinates)
        {
            return projectPosition(viewport, { station.x, station.y, station.z }, windowCoordinates);
        }

        Point projectEndpoint(const Viewport& viewport, const CargoDist::FlowAnalytics::Endpoint& endpoint, const bool windowCoordinates)
        {
            return projectPosition(viewport, { endpoint.position.x, endpoint.position.y, endpoint.z }, windowCoordinates);
        }

        double distanceToSegmentSquared(Point point, Point from, Point to)
        {
            const auto dx = static_cast<double>(to.x - from.x);
            const auto dy = static_cast<double>(to.y - from.y);
            const auto lengthSquared = dx * dx + dy * dy;
            if (lengthSquared == 0)
            {
                const auto pointDx = static_cast<double>(point.x - from.x);
                const auto pointDy = static_cast<double>(point.y - from.y);
                return pointDx * pointDx + pointDy * pointDy;
            }

            const auto projection = std::clamp(
                (static_cast<double>(point.x - from.x) * dx + static_cast<double>(point.y - from.y) * dy) / lengthSquared,
                0.0,
                1.0);
            const auto pointDx = static_cast<double>(point.x) - (from.x + projection * dx);
            const auto pointDy = static_cast<double>(point.y) - (from.y + projection * dy);
            return pointDx * pointDx + pointDy * pointDy;
        }

        int32_t toDisplayValue(uint64_t value)
        {
            return static_cast<int32_t>(std::min<uint64_t>(value, std::numeric_limits<int32_t>::max()));
        }

        int32_t calculatePercentage(const uint64_t demand, const uint64_t capacity)
        {
            const auto whole = demand / capacity;
            if (whole > std::numeric_limits<int32_t>::max() / 100)
            {
                return std::numeric_limits<int32_t>::max();
            }
            return toDisplayValue(whole * 100 + (demand % capacity) * 100 / capacity);
        }

        void offsetDirectionalLine(Point& from, Point& to)
        {
            const auto handedness = getGameState().trafficHandedness ? 1 : -1;
            if (std::abs(to.x - from.x) < std::abs(to.y - from.y))
            {
                const auto offset = (from.y > to.y ? 1 : -1) * handedness * 2;
                from.x += offset;
                to.x += offset;
            }
            else
            {
                const auto offset = (from.x < to.x ? 1 : -1) * handedness * 2;
                from.y += offset;
                to.y += offset;
            }
        }

        bool isVisibleStation(const ::OpenLoco::Station* station)
        {
            return station != nullptr && !station->empty() && (station->flags & StationFlags::flag_5) == StationFlags::none;
        }

        bool projectLinkEndpoints(const Viewport& viewport, const DisplayLink& link, const bool windowCoordinates, Point& fromPoint, Point& toPoint)
        {
            if (link.originEndpoint.has_value() && link.destinationEndpoint.has_value())
            {
                fromPoint = projectEndpoint(viewport, *link.originEndpoint, windowCoordinates);
                toPoint = projectEndpoint(viewport, *link.destinationEndpoint, windowCoordinates);
                return true;
            }

            const auto* from = StationManager::get(link.from);
            const auto* to = StationManager::get(link.to);
            if (!isVisibleStation(from) || !isVisibleStation(to))
            {
                return false;
            }
            fromPoint = projectStation(viewport, *from, windowCoordinates);
            toPoint = projectStation(viewport, *to, windowCoordinates);
            return true;
        }

        static void prepareDraw(Window& self)
        {
            if (const auto* cargoObj = ObjectManager::get<CargoObject>(_selectedCargo); cargoObj != nullptr)
            {
                auto args = FormatArguments(self.widgets[widx::cargo].textArgs);
                args.push(cargoObj->name);
            }
            {
                auto args = FormatArguments(self.widgets[widx::view].textArgs);
                args.push(getViewString(_selectedView));
            }
            {
                auto args = FormatArguments(self.widgets[widx::period].textArgs);
                args.push(getPeriodString(_selectedHorizon));
            }
            {
                auto args = FormatArguments(self.widgets[widx::scale].textArgs);
                args.push(_selectedScale == ScaleMode::absolute ? StringIds::cargo_flow_absolute : StringIds::cargo_flow_percentiles);
            }
            switch (_selectedView)
            {
                case ViewMode::serviceThroughput:
                    self.widgets[widx::saturationLabel].text = StringIds::cargo_flow_throughput_saturation;
                    break;
                case ViewMode::servicePlan:
                    self.widgets[widx::saturationLabel].text = StringIds::cargo_flow_planned_saturation;
                    break;
                case ViewMode::destinationDemand:
                    self.widgets[widx::saturationLabel].text = StringIds::cargo_flow_destination_saturation;
                    break;
                case ViewMode::destinationGaps:
                    self.widgets[widx::saturationLabel].text = StringIds::cargo_flow_gap_saturation;
                    break;
                case ViewMode::liveCommitments:
                    self.widgets[widx::saturationLabel].text = StringIds::cargo_flow_saturation;
                    break;
            }
            self.disabledWidgets &= ~((1ULL << widx::period) | (1ULL << widx::periodButton));
            if (_selectedView == ViewMode::liveCommitments)
            {
                self.disabledWidgets |= (1ULL << widx::period) | (1ULL << widx::periodButton);
            }
        }

        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            self.draw(drawingCtx);
            for (size_t i = 0; i < kSaturationColours.size(); ++i)
            {
                const auto left = kLegendLeft + static_cast<int16_t>(i) * kLegendCellWidth;
                drawingCtx.fillRect(left, kLegendTop, left + kLegendCellWidth - 2, kLegendTop + 7, kSaturationColours[i], Gfx::RectFlags::none);
            }

            auto tr = Gfx::TextRenderer(drawingCtx);
            tr.setCurrentFont(Gfx::Font::small);
            const auto textColour = self.getColour(WindowColour::secondary).opaque();
            const auto historicalService = _selectedView == ViewMode::serviceThroughput || _selectedView == ViewMode::servicePlan;
            if (historicalService && _snapshot.has_value() && _snapshot->links.empty())
            {
                tr.drawStringCentred({ kLegendLeft + kLegendCellWidth * 6, 107 }, textColour, StringIds::cargo_flow_collecting_history);
            }
            else
            {
                const auto absoluteLoad = _selectedView == ViewMode::serviceThroughput && _selectedScale == ScaleMode::absolute;
                tr.drawStringLeft({ kLegendLeft, 107 }, textColour, absoluteLoad ? StringIds::cargo_flow_zero_percent : StringIds::low);
                tr.drawStringCentred({ kLegendLeft + kLegendCellWidth * 6, 107 }, textColour, absoluteLoad ? StringIds::cargo_flow_fifty_percent : StringIds::medium);
                tr.drawStringRight({ kLegendLeft + kLegendCellWidth * 12 - 1, 107 }, textColour, absoluteLoad ? StringIds::cargo_flow_hundred_percent : StringIds::high);
            }
        }

        static void onClose(Window& self)
        {
            if (Input::isDropdownActive(self.type, self.number))
            {
                WindowManager::close(WindowType::dropdown);
                Input::resetFlag(Input::Flags::widgetPressed);
                Input::state(Input::State::normal);
            }
            _snapshot.reset();
            invalidateMainViewport();
        }

        static void onMouseUp(Window& self, WidgetIndex_t, const WidgetId id)
        {
            if (id == Widx::kCloseButton)
            {
                WindowManager::close(&self);
            }
        }

        static void onMouseDown(Window& self, WidgetIndex_t, const WidgetId id)
        {
            if (id == Widx::kCargoButton)
            {
                _dropdownCargoIds = getEnabledCargos();
                auto selectedIndex = -1;
                for (size_t index = 0; index < _dropdownCargoIds.size(); ++index)
                {
                    const auto cargo = _dropdownCargoIds[index];
                    const auto* cargoObj = ObjectManager::get<CargoObject>(cargo);
                    Dropdown::add(index, StringIds::dropdown_stringid, cargoObj->name);
                    if (cargo == _selectedCargo)
                    {
                        selectedIndex = static_cast<int16_t>(index);
                    }
                }

                const auto& widget = self.widgets[widx::cargo];
                Dropdown::showText(self.x + widget.left, self.y + widget.top, widget.width() - 4, widget.height(), self.getColour(WindowColour::secondary), _dropdownCargoIds.size(), 0);
                if (selectedIndex != -1)
                {
                    Dropdown::setItemSelected(selectedIndex);
                }
                return;
            }
            if (id == Widx::kViewButton)
            {
                for (size_t index = 0; index < kViewModes.size(); ++index)
                {
                    Dropdown::add(index, StringIds::dropdown_stringid, getViewString(kViewModes[index]));
                }
                const auto& widget = self.widgets[widx::view];
                Dropdown::showText(self.x + widget.left, self.y + widget.top, widget.width() - 4, widget.height(), self.getColour(WindowColour::secondary), kViewModes.size(), 0);
                Dropdown::setItemSelected(static_cast<size_t>(std::distance(kViewModes.begin(), std::find(kViewModes.begin(), kViewModes.end(), _selectedView))));
                return;
            }
            if (id == Widx::kPeriodButton && _selectedView != ViewMode::liveCommitments)
            {
                for (size_t index = 0; index < kHorizons.size(); ++index)
                {
                    Dropdown::add(index, StringIds::dropdown_stringid, getPeriodString(kHorizons[index]));
                }
                const auto& widget = self.widgets[widx::period];
                Dropdown::showText(self.x + widget.left, self.y + widget.top, widget.width() - 4, widget.height(), self.getColour(WindowColour::secondary), kHorizons.size(), 0);
                Dropdown::setItemSelected(static_cast<size_t>(std::distance(kHorizons.begin(), std::find(kHorizons.begin(), kHorizons.end(), _selectedHorizon))));
                return;
            }
            if (id == Widx::kScaleButton)
            {
                Dropdown::add(0, StringIds::dropdown_stringid, StringIds::cargo_flow_absolute);
                Dropdown::add(1, StringIds::dropdown_stringid, StringIds::cargo_flow_percentiles);
                const auto& widget = self.widgets[widx::scale];
                Dropdown::showText(self.x + widget.left, self.y + widget.top, widget.width() - 4, widget.height(), self.getColour(WindowColour::secondary), 2, 0);
                Dropdown::setItemSelected(static_cast<int16_t>(_selectedScale));
            }
        }

        static void onDropdown(Window& self, WidgetIndex_t, const WidgetId id, int16_t itemIndex)
        {
            if (itemIndex < 0)
            {
                return;
            }
            if (id == Widx::kCargoButton && static_cast<size_t>(itemIndex) < _dropdownCargoIds.size())
            {
                _selectedCargo = _dropdownCargoIds[itemIndex];
            }
            else if (id == Widx::kViewButton && static_cast<size_t>(itemIndex) < kViewModes.size())
            {
                _selectedView = kViewModes[itemIndex];
            }
            else if (id == Widx::kPeriodButton && static_cast<size_t>(itemIndex) < kHorizons.size())
            {
                _selectedHorizon = kHorizons[itemIndex];
            }
            else if (id == Widx::kScaleButton && itemIndex <= 1)
            {
                _selectedScale = static_cast<ScaleMode>(itemIndex);
                if (_snapshot.has_value())
                {
                    applyScale(*_snapshot, _selectedScale);
                }
            }
            else
            {
                return;
            }
            refreshSnapshot();
            self.invalidate();
            invalidateMainViewport();
        }

        static void onUpdate(Window& self)
        {
            const auto changed = refreshSnapshot();
            if (_selectedCargo == kNoCargo)
            {
                WindowManager::close(&self);
                return;
            }
            if (changed)
            {
                self.invalidate();
                invalidateMainViewport();
            }
        }

        static constexpr WindowEventList kEvents = {
            .onClose = onClose,
            .onMouseUp = onMouseUp,
            .onMouseDown = onMouseDown,
            .onDropdown = onDropdown,
            .onUpdate = onUpdate,
            .prepareDraw = prepareDraw,
            .draw = draw,
        };
    }

    bool hasEnabledCargo()
    {
        return !getEnabledCargos().empty();
    }

    bool isOpen()
    {
        return WindowManager::find(WindowType::cargoFlowOverlay) != nullptr;
    }

    Window* open()
    {
        if (auto* window = WindowManager::bringToFront(WindowType::cargoFlowOverlay); window != nullptr)
        {
            return window;
        }
        refreshSnapshot();
        if (_selectedCargo == kNoCargo)
        {
            return nullptr;
        }

        auto* window = WindowManager::createWindowCentred(WindowType::cargoFlowOverlay, kWindowSize, WindowFlags::none, kEvents);
        window->setWidgets(kWidgets);
        window->setColour(WindowColour::primary, ObjectManager::get<InterfaceSkinObject>()->windowTitlebarColour);
        window->setColour(WindowColour::secondary, ObjectManager::get<InterfaceSkinObject>()->windowOptionsColour);
        window->initScrollWidgets();
        window->callPrepareDraw();
        invalidateMainViewport();
        return window;
    }

    void toggle()
    {
        if (auto* window = WindowManager::find(WindowType::cargoFlowOverlay); window != nullptr)
        {
            WindowManager::close(window);
        }
        else
        {
            open();
        }
    }

    uint8_t getSaturationBucket(const uint64_t demand, const std::optional<uint64_t> capacity)
    {
        if (demand == 0)
        {
            return 0;
        }
        if (!capacity.has_value() || *capacity == 0)
        {
            return kSaturationColours.size() - 1;
        }

        const auto ratio = std::min<long double>(2.0L, static_cast<long double>(demand) / *capacity);
        return static_cast<uint8_t>(std::min<long double>(kSaturationColours.size() - 1, std::ceil(ratio * kSaturationColours.size() / 2) - 1));
    }

    PaletteIndex_t getSaturationColour(const uint8_t bucket)
    {
        return bucket >= kSaturationColours.size() ? PaletteIndex::black5 : kSaturationColours[bucket];
    }

    std::vector<uint8_t> calculateScaleBuckets(const std::span<const uint64_t> values, const ScaleMode mode, const std::optional<uint64_t> absoluteMaximum)
    {
        std::vector<uint8_t> buckets(values.size());
        if (values.empty())
        {
            return buckets;
        }
        if (mode == ScaleMode::absolute)
        {
            const auto maximum = absoluteMaximum.value_or(*std::ranges::max_element(values));
            if (maximum == 0)
            {
                return buckets;
            }
            for (size_t i = 0; i < values.size(); ++i)
            {
                if (values[i] != 0)
                {
                    const auto scaled = std::ceil(static_cast<long double>(values[i]) * (kSaturationColours.size() - 1) / maximum);
                    buckets[i] = static_cast<uint8_t>(std::clamp<long double>(scaled, 1, kSaturationColours.size() - 1));
                }
            }
            return buckets;
        }

        std::vector<uint64_t> sorted;
        sorted.reserve(values.size());
        std::ranges::copy_if(values, std::back_inserter(sorted), [](const auto value) { return value != 0; });
        std::ranges::sort(sorted);
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (values[i] != 0)
            {
                const auto rank = std::ranges::upper_bound(sorted, values[i]) - sorted.begin();
                buckets[i] = static_cast<uint8_t>((rank * (kSaturationColours.size() - 1) + sorted.size() - 1) / sorted.size());
            }
        }
        return buckets;
    }

    Projection project(const Viewport& viewport, const bool windowCoordinates)
    {
        if (!isOpen() || !isMainViewport(viewport))
        {
            return {};
        }
        refreshSnapshot();
        if (!_snapshot.has_value())
        {
            return {};
        }
        const auto& snapshot = *_snapshot;

        Projection projection;
        projection.links.reserve(snapshot.links.size());
        projection.markers.reserve(snapshot.endpoints.size());
        const auto viewportRect = windowCoordinates
            ? viewport.getUiRect()
            : Rect{
                  viewport.getViewOriginInRaster().x,
                  viewport.getViewOriginInRaster().y,
                  viewport.rasterWidth,
                  viewport.rasterHeight,
              };
        const auto visibleRect = Rect{ viewportRect.left() - 3, viewportRect.top() - 3, viewportRect.width() + 6, viewportRect.height() + 6 };
        for (const auto& link : snapshot.links)
        {
            Point fromPoint;
            Point toPoint;
            if (!projectLinkEndpoints(viewport, link, windowCoordinates, fromPoint, toPoint))
            {
                continue;
            }
            auto clippedFrom = fromPoint;
            auto clippedTo = toPoint;
            if (!clipLine(clippedFrom, clippedTo, visibleRect))
            {
                continue;
            }

            const auto disconnected = link.originEndpoint.has_value()
                && (link.gap == CargoDist::FlowAnalytics::GapReason::noRoute || link.gap == CargoDist::FlowAnalytics::GapReason::noStation);
            projection.links.push_back({ fromPoint, toPoint, kSaturationColours[link.scaleBucket], disconnected });
        }
        for (const auto& endpoint : snapshot.endpoints)
        {
            const auto position = projectEndpoint(viewport, endpoint, windowCoordinates);
            if (position.x < viewportRect.left() || position.x >= viewportRect.right()
                || position.y < viewportRect.top() || position.y >= viewportRect.bottom())
            {
                continue;
            }
            const auto colour = endpoint.supply != 0
                ? (endpoint.attraction != 0 ? PaletteIndex::yellowB : PaletteIndex::orangeA)
                : PaletteIndex::blue9;
            projection.markers.push_back({ position, colour });
        }
        return projection;
    }

    void drawLinks(Gfx::DrawingContext& drawingCtx, std::span<const ProjectedLink> links)
    {
        const auto clip = drawingCtx.currentRenderTarget().getUiRect();
        for (const auto& link : links)
        {
            if (link.dashed)
            {
                auto from = link.from;
                auto to = link.to;
                offsetDirectionalLine(from, to);
                if (!clipLine(from, to, clip))
                {
                    continue;
                }
                const auto steps = std::max(std::abs(to.x - from.x), std::abs(to.y - from.y));
                if (steps == 0)
                {
                    drawingCtx.drawLine(from, to, link.colour);
                    continue;
                }
                for (auto step = 0; step <= steps; step += 5)
                {
                    const auto end = std::min(step + 2, steps);
                    const auto pointAt = [&](const int32_t offset) {
                        return Point{
                            from.x + (to.x - from.x) * offset / steps,
                            from.y + (to.y - from.y) * offset / steps,
                        };
                    };
                    drawingCtx.drawLine(pointAt(step), pointAt(end), link.colour);
                }
                continue;
            }
            auto centreFrom = link.from;
            auto centreTo = link.to;
            if (clipLine(centreFrom, centreTo, clip))
            {
                drawingCtx.drawLine(centreFrom, centreTo, PaletteIndex::black3);
            }

            auto colourFrom = link.from;
            auto colourTo = link.to;
            offsetDirectionalLine(colourFrom, colourTo);
            if (clipLine(colourFrom, colourTo, clip))
            {
                drawingCtx.drawLine(colourFrom, colourTo, link.colour);
            }
        }
    }

    void drawMarkers(Gfx::DrawingContext& drawingCtx, const std::span<const ProjectedMarker> markers)
    {
        for (const auto& marker : markers)
        {
            drawingCtx.drawLine(marker.position - Point{ 2, 0 }, marker.position + Point{ 2, 0 }, PaletteIndex::black3);
            drawingCtx.drawLine(marker.position - Point{ 0, 2 }, marker.position + Point{ 0, 2 }, PaletteIndex::black3);
            drawingCtx.drawLine(marker.position - Point{ 1, 0 }, marker.position + Point{ 1, 0 }, marker.colour);
            drawingCtx.drawLine(marker.position - Point{ 0, 1 }, marker.position + Point{ 0, 1 }, marker.colour);
        }
    }

    bool setTooltip(const Viewport& viewport, Point cursor)
    {
        if (!isOpen() || !isMainViewport(viewport))
        {
            return false;
        }
        const auto viewportRect = viewport.getUiRect();
        if (cursor.x < viewportRect.left() || cursor.x >= viewportRect.right()
            || cursor.y < viewportRect.top() || cursor.y >= viewportRect.bottom())
        {
            return false;
        }
        refreshSnapshot();
        if (!_snapshot.has_value())
        {
            return false;
        }
        const auto& snapshot = *_snapshot;

        constexpr auto kTolerance = 5;
        const DisplayLink* hit = nullptr;
        const CargoDist::FlowAnalytics::Endpoint* hitEndpoint = nullptr;
        const ::OpenLoco::Station* hitFrom = nullptr;
        const ::OpenLoco::Station* hitTo = nullptr;
        auto nearestDistance = static_cast<double>(kTolerance * kTolerance + 1);
        for (const auto& link : snapshot.links)
        {
            Point fromPoint;
            Point toPoint;
            if (!projectLinkEndpoints(viewport, link, true, fromPoint, toPoint))
            {
                continue;
            }
            offsetDirectionalLine(fromPoint, toPoint);
            if (cursor.x < std::min(fromPoint.x, toPoint.x) - kTolerance || cursor.x > std::max(fromPoint.x, toPoint.x) + kTolerance
                || cursor.y < std::min(fromPoint.y, toPoint.y) - kTolerance || cursor.y > std::max(fromPoint.y, toPoint.y) + kTolerance)
            {
                continue;
            }

            const auto distance = distanceToSegmentSquared(cursor, fromPoint, toPoint);
            if (distance <= kTolerance * kTolerance && distance < nearestDistance)
            {
                hit = &link;
                hitFrom = link.originEndpoint.has_value() ? nullptr : StationManager::get(link.from);
                hitTo = link.destinationEndpoint.has_value() ? nullptr : StationManager::get(link.to);
                nearestDistance = distance;
            }
        }

        if (hit == nullptr)
        {
            for (const auto& endpoint : snapshot.endpoints)
            {
                const auto position = projectEndpoint(viewport, endpoint, true);
                const auto dx = cursor.x - position.x;
                const auto dy = cursor.y - position.y;
                const auto distance = static_cast<double>(dx * dx + dy * dy);
                if (distance <= kTolerance * kTolerance && distance < nearestDistance)
                {
                    hitEndpoint = &endpoint;
                    nearestDistance = distance;
                }
            }
        }

        const auto* cargo = ObjectManager::get<CargoObject>(snapshot.cargo);
        if (cargo == nullptr)
        {
            return false;
        }
        if (hit == nullptr)
        {
            if (hitEndpoint == nullptr)
            {
                return false;
            }
            auto args = FormatArguments::mapToolTip(StringIds::cargo_flow_endpoint_tooltip);
            args.push(cargo->name);
            args.push(hitEndpoint->name);
            args.push(hitEndpoint->town);
            args.push(static_cast<int32_t>(snapshot.horizonDays));
            args.push(toDisplayValue(hitEndpoint->supply));
            args.push(toDisplayValue(hitEndpoint->localDemand));
            return true;
        }

        if (hit->originEndpoint.has_value() && hit->destinationEndpoint.has_value())
        {
            const auto getGapString = [](const CargoDist::FlowAnalytics::GapReason gap) {
                switch (gap)
                {
                    case CargoDist::FlowAnalytics::GapReason::served:
                        return StringIds::cargo_flow_gap_served;
                    case CargoDist::FlowAnalytics::GapReason::capacityShortfall:
                        return StringIds::cargo_flow_gap_shortfall;
                    case CargoDist::FlowAnalytics::GapReason::noRoute:
                        return StringIds::cargo_flow_gap_no_route;
                    case CargoDist::FlowAnalytics::GapReason::noStation:
                        return StringIds::cargo_flow_gap_no_station;
                }
                return StringIds::cargo_flow_gap_no_route;
            };
            auto args = FormatArguments::mapToolTip(StringIds::cargo_flow_destination_tooltip);
            args.push(cargo->name);
            args.push(hit->originEndpoint->name);
            args.push(hit->originEndpoint->town);
            args.push(hit->destinationEndpoint->name);
            args.push(hit->destinationEndpoint->town);
            args.push(static_cast<int32_t>(snapshot.horizonDays));
            args.push(toDisplayValue(hit->demand));
            args.push(toDisplayValue(hit->capacity.value_or(0)));
            args.push(getGapString(hit->gap));
            args.push(toDisplayValue(hit->originEndpoint->localDemand));
            return true;
        }
        if (hitFrom == nullptr || hitTo == nullptr)
        {
            return false;
        }

        if (snapshot.view != ViewMode::liveCommitments)
        {
            if (hit->capacity.has_value() && *hit->capacity != 0)
            {
                auto args = FormatArguments::mapToolTip(snapshot.view == ViewMode::serviceThroughput ? StringIds::cargo_flow_throughput_tooltip : StringIds::cargo_flow_planned_tooltip);
                args.push(cargo->name);
                args.push(hitFrom->name);
                args.push(hitFrom->town);
                args.push(hitTo->name);
                args.push(hitTo->town);
                args.push(static_cast<int32_t>(snapshot.horizonDays));
                args.push(toDisplayValue(hit->demand));
                args.push(toDisplayValue(*hit->capacity));
                const auto percentage = calculatePercentage(hit->demand, *hit->capacity);
                args.push(snapshot.view == ViewMode::serviceThroughput ? std::min(100, percentage) : percentage);
            }
            else
            {
                auto args = FormatArguments::mapToolTip(StringIds::cargo_flow_history_no_capacity_tooltip);
                args.push(cargo->name);
                args.push(hitFrom->name);
                args.push(hitFrom->town);
                args.push(hitTo->name);
                args.push(hitTo->town);
                args.push(static_cast<int32_t>(snapshot.horizonDays));
                args.push(toDisplayValue(hit->demand));
            }
            return true;
        }

        const auto plannedDemand = toDisplayValue(hit->plannedDemand);
        const auto committedDemand = toDisplayValue(hit->committedDemand);
        const auto waitingDemand = toDisplayValue(hit->waitingDemand);
        const auto incomingDemand = toDisplayValue(hit->incomingDemand);
        if (hit->capacity.has_value() && *hit->capacity != 0)
        {
            auto args = FormatArguments::mapToolTip(StringIds::cargo_flow_tooltip);
            args.push(cargo->name);
            args.push(hitFrom->name);
            args.push(hitFrom->town);
            args.push(hitTo->name);
            args.push(hitTo->town);
            args.push(plannedDemand);
            args.push(committedDemand);
            args.push(waitingDemand);
            args.push(incomingDemand);
            args.push(toDisplayValue(*hit->capacity));
            args.push(calculatePercentage(hit->committedDemand, *hit->capacity));
        }
        else
        {
            auto args = FormatArguments::mapToolTip(StringIds::cargo_flow_tooltip_no_capacity);
            args.push(cargo->name);
            args.push(hitFrom->name);
            args.push(hitFrom->town);
            args.push(hitTo->name);
            args.push(hitTo->town);
            args.push(plannedDemand);
            args.push(committedDemand);
            args.push(waitingDemand);
            args.push(incomingDemand);
        }
        return true;
    }
}
