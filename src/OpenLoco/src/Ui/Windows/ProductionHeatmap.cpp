// SPDX-License-Identifier: MIT
#include "Ui/Windows/ProductionHeatmap.h"

#include "Game.h"
#include "GameStateFlags.h"
#include "Graphics/DrawingContext.h"
#include "Graphics/ImageIds.h"
#include "Graphics/TextRenderer.h"
#include "Input.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/StringIds.h"
#include "Map/BuildingElement.h"
#include "Map/TileManager.h"
#include "Objects/BuildingObject.h"
#include "Objects/CargoObject.h"
#include "Objects/IndustryObject.h"
#include "Objects/InterfaceSkinObject.h"
#include "Objects/ObjectManager.h"
#include "Scenario/ScenarioManager.h"
#include "Ui/Dropdown.h"
#include "Ui/ViewportInteraction.h"
#include "Ui/Widgets/CaptionWidget.h"
#include "Ui/Widgets/DropdownWidget.h"
#include "Ui/Widgets/FrameWidget.h"
#include "Ui/Widgets/ImageButtonWidget.h"
#include "Ui/Widgets/LabelWidget.h"
#include "Ui/Widgets/PanelWidget.h"
#include "Ui/WindowManager.h"
#include "Viewport.hpp"
#include "World/IndustryManager.h"
#include "World/Station.h"

#include <algorithm>
#include <limits>
#include <optional>

namespace OpenLoco::Ui::Windows::ProductionHeatmap
{
    namespace
    {
        constexpr uint8_t kNoCargo = 0xFF;
        constexpr Ui::Size kWindowSize = { 252, 110 };
        constexpr int16_t kLegendLeft = 80;
        constexpr int16_t kLegendTop = 76;
        constexpr int16_t kLegendCellWidth = 20;

        constexpr std::array<Colour, kBucketCount> kBucketColours = {
            Colour::darkBlue,
            Colour::blue,
            Colour::mutedTeal,
            Colour::green,
            Colour::yellow,
            Colour::amber,
            Colour::orange,
            Colour::red,
        };

        struct Snapshot
        {
            uint8_t cargo = kNoCargo;
            uint32_t mapRevision{};
            uint32_t simulationPeriod{};
            bool reducedProduction{};
            ScaleMode scaleMode = ScaleMode::percentiles;
            HeatmapLayers layers;
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
            modeLabel,
            mode,
            modeButton,
            scaleLabel,
            scale,
            scaleButton,
        };

        namespace Widx
        {
            constexpr WidgetId kFrame{ "frame" };
            constexpr WidgetId kCaption{ "caption" };
            constexpr WidgetId kCloseButton{ "close_button" };
            constexpr WidgetId kPanel{ "panel" };
            constexpr WidgetId kCargo{ "cargo" };
            constexpr WidgetId kCargoButton{ "cargo_button" };
            constexpr WidgetId kMode{ "mode" };
            constexpr WidgetId kModeButton{ "mode_button" };
            constexpr WidgetId kScale{ "scale" };
            constexpr WidgetId kScaleButton{ "scale_button" };
        }

        static constexpr auto kWidgets = makeWidgets(
            Widgets::Frame(Widx::kFrame, { 0, 0 }, kWindowSize, WindowColour::primary),
            Widgets::Caption(Widx::kCaption, { 1, 1 }, { kWindowSize.width - 2, 13 }, Widgets::Caption::Style::whiteText, WindowColour::primary, StringIds::production_heatmap),
            Widgets::ImageButton(Widx::kCloseButton, { kWindowSize.width - 15, 2 }, { 13, 13 }, WindowColour::primary, ImageIds::close_button, StringIds::tooltip_close_window),
            Widgets::Panel(Widx::kPanel, { 0, 15 }, { kWindowSize.width, kWindowSize.height - 15 }, WindowColour::secondary),
            Widgets::Label({ 6, 22 }, { 44, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::production_heatmap_cargo),
            Widgets::dropdownWidgets(Widx::kCargo, Widx::kCargoButton, { 50, 21 }, { 194, 12 }, WindowColour::secondary, StringIds::stringid, StringIds::tooltip_select_cargo_type),
            Widgets::Label({ 6, 39 }, { 44, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::production_heatmap_mode),
            Widgets::dropdownWidgets(Widx::kMode, Widx::kModeButton, { 50, 38 }, { 194, 12 }, WindowColour::secondary, StringIds::stringid),
            Widgets::Label({ 6, 56 }, { 44, 12 }, WindowColour::secondary, ContentAlign::left, StringIds::production_heatmap_scale),
            Widgets::dropdownWidgets(Widx::kScale, Widx::kScaleButton, { 50, 55 }, { 194, 12 }, WindowColour::secondary, StringIds::stringid));

        uint8_t _selectedCargo = kNoCargo;
        Mode _selectedMode = Mode::stationPotential;
        ScaleMode _selectedScale = ScaleMode::percentiles;
        std::optional<Snapshot> _snapshot;
        std::vector<uint8_t> _dropdownCargoIds;
        uint32_t _pendingMapRevision{};
        uint8_t _stableMapRevisionUpdates{};

        size_t getIndex(const World::TilePos2& pos, const uint16_t width)
        {
            return static_cast<size_t>(pos.y) * width + pos.x;
        }

        bool isInBounds(const World::TilePos2& pos, const uint16_t width, const uint16_t height)
        {
            return pos.x >= 0 && pos.y >= 0 && pos.x < width && pos.y < height;
        }

        void saturatingAdd(uint64_t& target, const uint64_t value)
        {
            target = std::numeric_limits<uint64_t>::max() - target < value ? std::numeric_limits<uint64_t>::max() : target + value;
        }

        std::vector<World::TilePos2> normaliseFootprint(std::span<const World::TilePos2> footprint, const uint16_t width, const uint16_t height)
        {
            std::vector<World::TilePos2> result;
            result.reserve(footprint.size());
            for (const auto& tile : footprint)
            {
                if (isInBounds(tile, width, height))
                {
                    result.push_back(tile);
                }
            }
            std::sort(result.begin(), result.end(), [width](const auto& lhs, const auto& rhs) { return getIndex(lhs, width) < getIndex(rhs, width); });
            result.erase(std::unique(result.begin(), result.end()), result.end());
            return result;
        }

        void finishLayer(HeatmapLayer& layer, const ScaleMode scaleMode)
        {
            const auto thresholds = calculatePercentileThresholds(layer.values);
            uint64_t minimum = 0;
            uint64_t maximum = 0;
            for (const auto value : layer.values)
            {
                if (value != 0 && (minimum == 0 || value < minimum))
                {
                    minimum = value;
                }
                maximum = std::max(maximum, value);
            }
            layer.buckets.resize(layer.values.size());
            for (size_t i = 0; i < layer.values.size(); ++i)
            {
                layer.buckets[i] = scaleMode == ScaleMode::percentiles ? getPercentileBucket(layer.values[i], thresholds) : getLinearBucket(layer.values[i], minimum, maximum);
            }
        }

        std::vector<uint8_t> getEnabledCargos()
        {
            std::vector<uint8_t> cargos;
            for (uint16_t cargo = 0; cargo < ObjectManager::getMaxObjects(ObjectType::cargo); ++cargo)
            {
                if (ObjectManager::get<CargoObject>(cargo) != nullptr)
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

        void addSource(std::vector<ProductionSource>& sources, const uint64_t production, std::vector<World::TilePos2>&& footprint)
        {
            if (production != 0 && !footprint.empty())
            {
                sources.push_back({ production, std::move(footprint) });
            }
        }

        std::vector<ProductionSource> collectProductionSources(const uint8_t cargo)
        {
            std::vector<ProductionSource> sources;
            const auto reducedProduction = Game::hasFlags(GameStateFlags::unk2);
            for (tile_coord_t y = 0; y < World::kMapRows; ++y)
            {
                for (tile_coord_t x = 0; x < World::kMapColumns; ++x)
                {
                    const World::TilePos2 tilePos{ x, y };
                    const auto tile = World::TileManager::get(tilePos);
                    for (const auto& element : tile)
                    {
                        if (element.isGhost() || element.type() != World::ElementType::building)
                        {
                            continue;
                        }
                        const auto& building = element.get<World::BuildingElement>();
                        if (building.sequenceIndex() != 0 || building.isMiscBuilding() || !building.isConstructed())
                        {
                            continue;
                        }
                        const auto* object = building.getObject();
                        if (object == nullptr)
                        {
                            continue;
                        }

                        uint64_t production = 0;
                        for (uint8_t i = 0; i < 2; ++i)
                        {
                            if (object->producedCargoType[i] == cargo)
                            {
                                production += getBuildingMonthlyProductionEstimateScaled(object->producedQuantity[i], reducedProduction);
                            }
                        }

                        std::vector<World::TilePos2> footprint;
                        const auto origin = World::toWorldSpace(tilePos);
                        for (const auto& offset : getBuildingTileOffsets(object->hasFlags(BuildingObjectFlags::largeTile)))
                        {
                            footprint.push_back(World::toTileSpace(origin + offset.pos));
                        }
                        addSource(sources, production, std::move(footprint));
                    }
                }
            }

            for (const auto& industry : IndustryManager::industries())
            {
                if (industry.empty() || industry.hasFlags(IndustryFlags::isGhost) || industry.under_construction != kIndustryConstructionComplete)
                {
                    continue;
                }
                const auto* object = industry.getObject();
                if (object == nullptr)
                {
                    continue;
                }

                uint64_t production = 0;
                for (uint8_t i = 0; i < 2; ++i)
                {
                    if (object->producedCargoType[i] == cargo)
                    {
                        production += static_cast<uint64_t>(industry.producedCargoQuantityPreviousMonth[i]) * kMonthlyProductionEstimateDenominator;
                    }
                }

                std::vector<World::TilePos2> footprint;
                for (uint8_t i = 0; i < industry.numTiles; ++i)
                {
                    const auto& anchor = industry.tiles[i];
                    const auto multiTile = (anchor.z & (1U << 15)) != 0;
                    const World::Pos2 origin{ anchor.x, anchor.y };
                    for (const auto& offset : getBuildingTileOffsets(multiTile))
                    {
                        footprint.push_back(World::toTileSpace(origin + offset.pos));
                    }
                }
                addSource(sources, production, std::move(footprint));
            }
            return sources;
        }

        bool refreshSnapshot()
        {
            if (!selectValidCargo() || !Game::hasFlags(GameStateFlags::tileManagerLoaded))
            {
                const auto changed = _snapshot.has_value();
                _snapshot.reset();
                return changed;
            }

            const auto mapRevision = World::TileManager::getMapRevision();
            const auto simulationPeriod = ScenarioManager::getScenarioTicks() / 1024;
            const auto reducedProduction = Game::hasFlags(GameStateFlags::unk2);
            if (_snapshot.has_value() && _snapshot->cargo == _selectedCargo && _snapshot->simulationPeriod == simulationPeriod
                && _snapshot->reducedProduction == reducedProduction && _snapshot->scaleMode == _selectedScale)
            {
                if (_snapshot->mapRevision == mapRevision)
                {
                    return false;
                }
                if (_pendingMapRevision != mapRevision)
                {
                    _pendingMapRevision = mapRevision;
                    _stableMapRevisionUpdates = 0;
                    return false;
                }
                if (++_stableMapRevisionUpdates < 8)
                {
                    return false;
                }
            }

            Snapshot snapshot{};
            snapshot.cargo = _selectedCargo;
            snapshot.mapRevision = mapRevision;
            snapshot.simulationPeriod = simulationPeriod;
            snapshot.reducedProduction = reducedProduction;
            snapshot.scaleMode = _selectedScale;
            const auto sources = collectProductionSources(_selectedCargo);
            snapshot.layers = buildProductionLayers(sources, World::kMapColumns, World::kMapRows, kCatchmentRadius, true, _selectedScale);
            _snapshot = std::move(snapshot);
            _pendingMapRevision = mapRevision;
            _stableMapRevisionUpdates = 0;
            return true;
        }

        const HeatmapLayer* getActiveLayer()
        {
            if (!_snapshot.has_value())
            {
                return nullptr;
            }
            return _selectedMode == Mode::physicalProduction ? &_snapshot->layers.physical : &_snapshot->layers.stationPotential;
        }

        StringId getModeStringId(const Mode mode)
        {
            return mode == Mode::physicalProduction ? StringIds::production_heatmap_physical : StringIds::production_heatmap_station_potential;
        }

        void invalidateMainViewport()
        {
            if (auto* main = WindowManager::getMainWindow(); main != nullptr)
            {
                main->invalidate();
            }
        }

        void setViewportEnabled(const bool enabled)
        {
            auto* main = WindowManager::getMainWindow();
            if (main == nullptr || main->viewports[0] == nullptr)
            {
                return;
            }
            if (enabled)
            {
                main->viewports[0]->flags |= ViewportFlags::production_heatmap;
            }
            else
            {
                main->viewports[0]->flags &= ~ViewportFlags::production_heatmap;
            }
            main->invalidate();
        }

        static void prepareDraw(Window& self)
        {
            if (const auto* cargo = ObjectManager::get<CargoObject>(_selectedCargo); cargo != nullptr)
            {
                auto args = FormatArguments(self.widgets[widx::cargo].textArgs);
                args.push(cargo->name);
            }
            auto modeArgs = FormatArguments(self.widgets[widx::mode].textArgs);
            modeArgs.push(getModeStringId(_selectedMode));
            auto scaleArgs = FormatArguments(self.widgets[widx::scale].textArgs);
            scaleArgs.push(_selectedScale == ScaleMode::percentiles ? StringIds::production_heatmap_percentiles : StringIds::production_heatmap_linear);
        }

        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            self.draw(drawingCtx);
            for (size_t i = 0; i < kBucketColours.size(); ++i)
            {
                const auto left = kLegendLeft + static_cast<int16_t>(i) * kLegendCellWidth;
                drawingCtx.fillRect(left, kLegendTop, left + kLegendCellWidth - 2, kLegendTop + 7, Colours::getShade(kBucketColours[i], 7), Gfx::RectFlags::none);
            }
            auto tr = Gfx::TextRenderer(drawingCtx);
            tr.setCurrentFont(Gfx::Font::small);
            const auto colour = self.getColour(WindowColour::secondary).opaque();
            constexpr auto kLegendLabelY = kLegendTop + 13;
            tr.drawStringLeft({ kLegendLeft, kLegendLabelY }, colour, StringIds::low);
            tr.drawStringCentred({ kLegendLeft + kLegendCellWidth * 4, kLegendLabelY }, colour, StringIds::medium);
            tr.drawStringRight({ kLegendLeft + kLegendCellWidth * kBucketCount - 1, kLegendLabelY }, colour, StringIds::high);
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
            setViewportEnabled(false);
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
                auto selected = -1;
                for (size_t i = 0; i < _dropdownCargoIds.size(); ++i)
                {
                    const auto cargo = _dropdownCargoIds[i];
                    Dropdown::add(i, StringIds::dropdown_stringid, ObjectManager::get<CargoObject>(cargo)->name);
                    if (cargo == _selectedCargo)
                    {
                        selected = static_cast<int16_t>(i);
                    }
                }
                const auto& widget = self.widgets[widx::cargo];
                Dropdown::showText(self.x + widget.left, self.y + widget.top, widget.width() - 4, widget.height(), self.getColour(WindowColour::secondary), _dropdownCargoIds.size(), 0);
                if (selected != -1)
                {
                    Dropdown::setItemSelected(selected);
                }
            }
            else if (id == Widx::kModeButton)
            {
                Dropdown::add(0, StringIds::dropdown_stringid, StringIds::production_heatmap_physical);
                Dropdown::add(1, StringIds::dropdown_stringid, StringIds::production_heatmap_station_potential);
                const auto& widget = self.widgets[widx::mode];
                Dropdown::showText(self.x + widget.left, self.y + widget.top, widget.width() - 4, widget.height(), self.getColour(WindowColour::secondary), 2, 0);
                Dropdown::setItemSelected(static_cast<int16_t>(_selectedMode));
            }
            else if (id == Widx::kScaleButton)
            {
                Dropdown::add(0, StringIds::dropdown_stringid, StringIds::production_heatmap_percentiles);
                Dropdown::add(1, StringIds::dropdown_stringid, StringIds::production_heatmap_linear);
                const auto& widget = self.widgets[widx::scale];
                Dropdown::showText(self.x + widget.left, self.y + widget.top, widget.width() - 4, widget.height(), self.getColour(WindowColour::secondary), 2, 0);
                Dropdown::setItemSelected(static_cast<int16_t>(_selectedScale));
            }
        }

        static void onDropdown(Window& self, WidgetIndex_t, const WidgetId id, const int16_t itemIndex)
        {
            if (id == Widx::kCargoButton && itemIndex >= 0 && static_cast<size_t>(itemIndex) < _dropdownCargoIds.size())
            {
                _selectedCargo = _dropdownCargoIds[itemIndex];
                refreshSnapshot();
            }
            else if (id == Widx::kModeButton && itemIndex >= 0 && itemIndex <= 1)
            {
                _selectedMode = static_cast<Mode>(itemIndex);
            }
            else if (id == Widx::kScaleButton && itemIndex >= 0 && itemIndex <= 1)
            {
                _selectedScale = static_cast<ScaleMode>(itemIndex);
                refreshSnapshot();
            }
            else
            {
                return;
            }
            self.invalidate();
            invalidateMainViewport();
        }

        static void onUpdate(Window& self)
        {
            const auto changed = refreshSnapshot();
            if (_selectedCargo == kNoCargo || !_snapshot.has_value())
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

    std::array<uint64_t, kBucketCount - 1> calculatePercentileThresholds(const std::span<const uint64_t> values)
    {
        std::vector<uint64_t> nonzero;
        nonzero.reserve(values.size());
        std::copy_if(values.begin(), values.end(), std::back_inserter(nonzero), [](const auto value) { return value != 0; });
        std::sort(nonzero.begin(), nonzero.end());

        std::array<uint64_t, kBucketCount - 1> thresholds{};
        if (nonzero.empty())
        {
            return thresholds;
        }
        for (size_t i = 1; i < kBucketCount; ++i)
        {
            auto index = (i * nonzero.size() + kBucketCount - 1) / kBucketCount;
            while (index < nonzero.size() && nonzero[index] == nonzero[index - 1])
            {
                ++index;
            }
            thresholds[i - 1] = index < nonzero.size() ? nonzero[index] : nonzero.back();
        }
        return thresholds;
    }

    uint8_t getPercentileBucket(const uint64_t value, const std::array<uint64_t, kBucketCount - 1>& thresholds)
    {
        if (value == 0)
        {
            return 0;
        }
        auto bucket = uint8_t{ 1 };
        for (const auto threshold : thresholds)
        {
            bucket += threshold != 0 && value >= threshold;
        }
        return bucket;
    }

    uint8_t getLinearBucket(const uint64_t value, const uint64_t minimum, const uint64_t maximum)
    {
        if (value == 0 || maximum == 0)
        {
            return 0;
        }
        if (minimum >= maximum || value <= minimum)
        {
            return 1;
        }
        const auto range = maximum - minimum;
        const auto proportion = static_cast<long double>(value - minimum) / static_cast<long double>(range);
        return static_cast<uint8_t>(1 + proportion * (kBucketCount - 1));
    }

    HeatmapLayers buildProductionLayers(const std::span<const ProductionSource> sources, const uint16_t width, const uint16_t height, const uint8_t catchmentRadius, const bool excludeNonDrawableBorder, const ScaleMode scaleMode)
    {
        HeatmapLayers result{};
        const auto cellCount = static_cast<size_t>(width) * height;
        result.physical.values.resize(cellCount);
        result.stationPotential.values.resize(cellCount);
        std::vector<uint32_t> sourceStamp(cellCount);
        uint32_t stamp = 0;

        for (const auto& source : sources)
        {
            const auto footprint = normaliseFootprint(source.footprint, width, height);
            if (source.monthlyProductionScaled == 0 || footprint.empty())
            {
                continue;
            }

            const auto share = source.monthlyProductionScaled / footprint.size();
            auto remainder = source.monthlyProductionScaled % footprint.size();
            for (const auto& tile : footprint)
            {
                const auto extra = remainder != 0;
                remainder -= extra;
                saturatingAdd(result.physical.values[getIndex(tile, width)], share + extra);
            }

            if (++stamp == 0)
            {
                std::fill(sourceStamp.begin(), sourceStamp.end(), 0);
                ++stamp;
            }
            for (const auto& tile : footprint)
            {
                const auto minX = std::max<int32_t>(0, tile.x - catchmentRadius);
                const auto maxX = std::min<int32_t>(width - 1, tile.x + catchmentRadius);
                const auto minY = std::max<int32_t>(0, tile.y - catchmentRadius);
                const auto maxY = std::min<int32_t>(height - 1, tile.y + catchmentRadius);
                for (auto y = minY; y <= maxY; ++y)
                {
                    for (auto x = minX; x <= maxX; ++x)
                    {
                        const auto index = static_cast<size_t>(y) * width + x;
                        if (sourceStamp[index] != stamp)
                        {
                            sourceStamp[index] = stamp;
                            saturatingAdd(result.stationPotential.values[index], source.monthlyProductionScaled);
                        }
                    }
                }
            }
        }

        if (excludeNonDrawableBorder)
        {
            for (uint16_t y = 0; y < height; ++y)
            {
                for (uint16_t x = 0; x < width; ++x)
                {
                    if (x < 1 || y < 1 || x >= width - 1 || y >= height - 1)
                    {
                        const auto index = static_cast<size_t>(y) * width + x;
                        result.physical.values[index] = 0;
                        result.stationPotential.values[index] = 0;
                    }
                }
            }
        }
        finishLayer(result.physical, scaleMode);
        finishLayer(result.stationPotential, scaleMode);
        return result;
    }

    bool hasEnabledCargo()
    {
        return !getEnabledCargos().empty();
    }

    bool isOpen()
    {
        return WindowManager::find(WindowType::productionHeatmap) != nullptr;
    }

    Window* open()
    {
        if (auto* window = WindowManager::bringToFront(WindowType::productionHeatmap); window != nullptr)
        {
            setViewportEnabled(true);
            return window;
        }
        refreshSnapshot();
        if (_selectedCargo == kNoCargo || !_snapshot.has_value())
        {
            return nullptr;
        }
        auto* window = WindowManager::createWindowCentred(WindowType::productionHeatmap, kWindowSize, WindowFlags::none, kEvents);
        window->setWidgets(kWidgets);
        window->setColour(WindowColour::primary, ObjectManager::get<InterfaceSkinObject>()->windowTitlebarColour);
        window->setColour(WindowColour::secondary, ObjectManager::get<InterfaceSkinObject>()->windowOptionsColour);
        window->initScrollWidgets();
        window->callPrepareDraw();
        setViewportEnabled(true);
        return window;
    }

    void toggle()
    {
        if (auto* window = WindowManager::find(WindowType::productionHeatmap); window != nullptr)
        {
            WindowManager::close(window);
        }
        else
        {
            open();
        }
    }

    uint8_t getTileBucket(const World::Pos2& pos)
    {
        const auto* layer = getActiveLayer();
        const auto tile = World::toTileSpace(pos);
        if (layer == nullptr || !World::validCoords(tile))
        {
            return 0;
        }
        return layer->buckets[getIndex(tile, World::kMapColumns)];
    }

    uint64_t getTileValue(const World::Pos2& pos)
    {
        const auto* layer = getActiveLayer();
        const auto tile = World::toTileSpace(pos);
        if (layer == nullptr || !World::validCoords(tile))
        {
            return 0;
        }
        return layer->values[getIndex(tile, World::kMapColumns)];
    }

    Colour getBucketColour(const uint8_t bucket)
    {
        return bucket == 0 || bucket > kBucketColours.size() ? Colour::black : kBucketColours[bucket - 1];
    }

    bool setTooltip(const Viewport& viewport, const Point cursor)
    {
        if (!isOpen() || (viewport.flags & ViewportFlags::production_heatmap) == ViewportFlags::none)
        {
            return false;
        }
        const auto surface = ViewportInteraction::getSurfaceLocFromUi(cursor);
        if (!surface.has_value() || surface->second != &viewport)
        {
            return false;
        }
        const auto value = getTileValue(surface->first);
        const auto bucket = getTileBucket(surface->first);
        const auto* cargo = ObjectManager::get<CargoObject>(_selectedCargo);
        if (value == 0 || bucket == 0 || cargo == nullptr)
        {
            return false;
        }

        auto args = FormatArguments::mapToolTip(StringIds::production_heatmap_tooltip);
        args.push(cargo->name);
        args.push(getModeStringId(_selectedMode));
        const auto maxScaledValue = static_cast<uint64_t>(std::numeric_limits<int32_t>::max()) * kMonthlyProductionEstimateDenominator / 100;
        const auto hundredths = value >= maxScaledValue ? std::numeric_limits<int32_t>::max() : static_cast<int32_t>((value * 100 + kMonthlyProductionEstimateDenominator / 2) / kMonthlyProductionEstimateDenominator);
        args.push<int32_t>(std::max(hundredths, 1));
        args.push<int32_t>(bucket);
        args.push<int32_t>(kBucketCount);
        return true;
    }
}
