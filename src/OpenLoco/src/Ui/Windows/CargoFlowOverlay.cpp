// SPDX-License-Identifier: MIT
#include "Ui/Windows/CargoFlowOverlay.h"

#include "CargoDist/CargoDist.h"
#include "GameState.h"
#include "Graphics/DrawingContext.h"
#include "Graphics/ImageIds.h"
#include "Graphics/RenderTarget.h"
#include "Graphics/TextRenderer.h"
#include "Input.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/StringIds.h"
#include "Map/Tile.h"
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
        constexpr Ui::Size kWindowSize = { 242, 74 };
        constexpr int16_t kLegendLeft = 78;
        constexpr int16_t kLegendTop = 41;
        constexpr int16_t kLegendCellWidth = 13;

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

        struct Snapshot
        {
            uint8_t cargo = kNoCargo;
            uint64_t routingRevision{};
            std::vector<CargoDist::PlannedServiceEdge> links;
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
        }

        static constexpr auto kWidgets = makeWidgets(
            Widgets::Frame(Widx::kFrame, { 0, 0 }, kWindowSize, WindowColour::primary),
            Widgets::Caption(Widx::kCaption, { 1, 1 }, { kWindowSize.width - 2, 13 }, Widgets::Caption::Style::whiteText, WindowColour::primary, StringIds::cargo_flow_overlay),
            Widgets::ImageButton(Widx::kCloseButton, { kWindowSize.width - 15, 2 }, { 13, 13 }, WindowColour::primary, ImageIds::close_button, StringIds::tooltip_close_window),
            Widgets::Panel(Widx::kPanel, { 0, 15 }, { kWindowSize.width, kWindowSize.height - 15 }, WindowColour::secondary),
            Widgets::Label({ 6, 22 }, { 44, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::cargo_flow_cargo),
            Widgets::dropdownWidgets(Widx::kCargo, Widx::kCargoButton, { 50, 21 }, { 184, 12 }, WindowColour::secondary, StringIds::stringid, StringIds::tooltip_select_cargo_type),
            Widgets::Label({ 6, 40 }, { 70, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::cargo_flow_saturation));

        uint8_t _selectedCargo = kNoCargo;
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

        bool refreshSnapshot()
        {
            if (!selectValidCargo())
            {
                const auto changed = _snapshot.has_value();
                _snapshot.reset();
                return changed;
            }

            const auto revision = CargoDist::getStateConst().routingRevision;
            if (_snapshot.has_value() && _snapshot->cargo == _selectedCargo && _snapshot->routingRevision == revision)
            {
                return false;
            }

            _snapshot = Snapshot{
                _selectedCargo,
                revision,
                CargoDist::getPlannedServiceEdges(_selectedCargo),
            };
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

        Point projectStation(const Viewport& viewport, const ::OpenLoco::Station& station, bool windowCoordinates)
        {
            const auto projected = World::gameToScreen(World::Pos3{ station.x, station.y, station.z }, viewport.getRotation());
            if (windowCoordinates)
            {
                const auto viewOrigin = Point{
                    viewport.zoom.applyInversedTo(viewport.viewX),
                    viewport.zoom.applyInversedTo(viewport.viewY),
                };
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

        int32_t calculatePercentage(uint64_t plannedDemand, uint32_t capacity)
        {
            const auto whole = plannedDemand / capacity;
            if (whole > std::numeric_limits<int32_t>::max() / 100)
            {
                return std::numeric_limits<int32_t>::max();
            }
            return toDisplayValue(whole * 100 + (plannedDemand % capacity) * 100 / capacity);
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

        static void prepareDraw(Window& self)
        {
            if (const auto* cargoObj = ObjectManager::get<CargoObject>(_selectedCargo); cargoObj != nullptr)
            {
                auto args = FormatArguments(self.widgets[widx::cargo].textArgs);
                args.push(cargoObj->name);
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
            tr.drawStringLeft({ kLegendLeft, 53 }, Colour::black, StringIds::cargo_flow_unused);
            tr.drawStringCentred({ kLegendLeft + kLegendCellWidth * 6, 53 }, Colour::black, StringIds::cargo_flow_saturated);
            tr.drawStringRight({ kLegendLeft + kLegendCellWidth * 12 - 1, 53 }, Colour::black, StringIds::cargo_flow_overloaded);
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
            if (id != Widx::kCargoButton)
            {
                return;
            }

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
        }

        static void onDropdown(Window& self, WidgetIndex_t, const WidgetId id, int16_t itemIndex)
        {
            if (id != Widx::kCargoButton || itemIndex < 0 || static_cast<size_t>(itemIndex) >= _dropdownCargoIds.size())
            {
                return;
            }

            _selectedCargo = _dropdownCargoIds[itemIndex];
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

    uint8_t getSaturationBucket(uint64_t plannedDemand, std::optional<uint32_t> capacity)
    {
        if (plannedDemand == 0)
        {
            return 0;
        }
        if (!capacity.has_value() || *capacity == 0)
        {
            return kSaturationColours.size() - 1;
        }

        const auto doubledCapacity = static_cast<uint64_t>(*capacity) * 2;
        const auto cappedDemand = std::min(plannedDemand, doubledCapacity);
        return (cappedDemand * kSaturationColours.size() - 1) / doubledCapacity;
    }

    std::vector<ProjectedLink> projectLinks(const Viewport& viewport, bool windowCoordinates)
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

        std::vector<ProjectedLink> projectedLinks;
        projectedLinks.reserve(snapshot.links.size());
        const auto viewportRect = windowCoordinates
            ? viewport.getUiRect()
            : Rect{
                  viewport.zoom.applyInversedTo(viewport.viewX),
                  viewport.zoom.applyInversedTo(viewport.viewY),
                  viewport.rasterWidth,
                  viewport.rasterHeight,
              };
        const auto visibleRect = Rect{ viewportRect.left() - 3, viewportRect.top() - 3, viewportRect.width() + 6, viewportRect.height() + 6 };
        for (const auto& link : snapshot.links)
        {
            const auto* from = StationManager::get(link.from);
            const auto* to = StationManager::get(link.to);
            if (!isVisibleStation(from) || !isVisibleStation(to))
            {
                continue;
            }

            const auto fromPoint = projectStation(viewport, *from, windowCoordinates);
            const auto toPoint = projectStation(viewport, *to, windowCoordinates);
            auto clippedFrom = fromPoint;
            auto clippedTo = toPoint;
            if (!clipLine(clippedFrom, clippedTo, visibleRect))
            {
                continue;
            }

            projectedLinks.push_back({
                fromPoint,
                toPoint,
                kSaturationColours[getSaturationBucket(link.plannedDemand, link.capacity)],
            });
        }
        return projectedLinks;
    }

    void drawLinks(Gfx::DrawingContext& drawingCtx, std::span<const ProjectedLink> links)
    {
        const auto clip = drawingCtx.currentRenderTarget().getUiRect();
        for (const auto& link : links)
        {
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

    bool setTooltip(const Viewport& viewport, Point cursor)
    {
        if (!isOpen() || !isMainViewport(viewport))
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
        const CargoDist::PlannedServiceEdge* hit = nullptr;
        const ::OpenLoco::Station* hitFrom = nullptr;
        const ::OpenLoco::Station* hitTo = nullptr;
        auto nearestDistance = static_cast<double>(kTolerance * kTolerance + 1);
        for (const auto& link : snapshot.links)
        {
            const auto* from = StationManager::get(link.from);
            const auto* to = StationManager::get(link.to);
            if (!isVisibleStation(from) || !isVisibleStation(to))
            {
                continue;
            }

            auto fromPoint = projectStation(viewport, *from, true);
            auto toPoint = projectStation(viewport, *to, true);
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
                hitFrom = from;
                hitTo = to;
                nearestDistance = distance;
            }
        }

        const auto* cargo = ObjectManager::get<CargoObject>(snapshot.cargo);
        if (hit == nullptr || cargo == nullptr)
        {
            return false;
        }

        const auto plannedDemand = toDisplayValue(hit->plannedDemand);
        if (hit->capacity.has_value() && *hit->capacity != 0)
        {
            auto args = FormatArguments::mapToolTip(StringIds::cargo_flow_tooltip);
            args.push(cargo->name);
            args.push(hitFrom->name);
            args.push(hitFrom->town);
            args.push(hitTo->name);
            args.push(hitTo->town);
            args.push(plannedDemand);
            args.push(toDisplayValue(*hit->capacity));
            args.push(calculatePercentage(hit->plannedDemand, *hit->capacity));
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
        }
        return true;
    }
}
