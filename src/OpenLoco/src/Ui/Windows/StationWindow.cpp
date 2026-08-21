#include "Config.h"
#include "Entities/EntityManager.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/General/RenameStation.h"
#include "Graphics/Colour.h"
#include "Graphics/Gfx.h"
#include "Graphics/ImageIds.h"
#include "Graphics/RenderTarget.h"
#include "Graphics/TextRenderer.h"
#include "Input.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/Formatting.h"
#include "Localisation/StringIds.h"
#include "Map/MapSelection.h"
#include "Map/TileLoop.hpp"
#include "Map/TileManager.h"
#include "Objects/CargoObject.h"
#include "Objects/InterfaceSkinObject.h"
#include "Objects/ObjectManager.h"
#include "Ui/CargoRouteTree.h"
#include "Ui/Dropdown.h"
#include "Ui/ScrollView.h"
#include "Ui/ToolManager.h"
#include "Ui/Widget.h"
#include "Ui/Widgets/CaptionWidget.h"
#include "Ui/Widgets/DropdownWidget.h"
#include "Ui/Widgets/FrameWidget.h"
#include "Ui/Widgets/ImageButtonWidget.h"
#include "Ui/Widgets/LabelWidget.h"
#include "Ui/Widgets/PanelWidget.h"
#include "Ui/Widgets/ScrollViewWidget.h"
#include "Ui/Widgets/TabWidget.h"
#include "Ui/Widgets/ViewportWidget.h"
#include "Ui/WindowManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleDraw.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "ViewportManager.h"
#include "World/CompanyManager.h"
#include "World/StationManager.h"
#include <OpenLoco/CargoDist/CargoDist.h>
#include <OpenLoco/Utility/String.hpp>
#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <set>
#include <span>
#include <vector>

using namespace OpenLoco::World;

namespace OpenLoco::Ui::Windows::Station
{
    static StationId _lastSelectedStation; // 0x0112C786

    struct CargoWindowState
    {
        uint32_t expanded{};
        CargoRouteTree::GroupOrder groupOrder = CargoRouteTree::GroupOrder::sourceDestinationVia;
        CargoRouteTree::SortMode sortMode = CargoRouteTree::SortMode::amountWaiting;
        std::map<uint8_t, std::set<CargoRouteTree::GroupKey>> expandedGroups;
        uint64_t cargoRevision = std::numeric_limits<uint64_t>::max();
        uint64_t routingRevision = std::numeric_limits<uint64_t>::max();
        std::map<uint8_t, std::vector<CargoDist::CargoRouteNode>> routeTrees;
    };

    static std::map<StationId, CargoWindowState> _cargoWindowStates;

    using Vehicles::VehicleHead;

    namespace Common
    {
        static constexpr Ui::Size kMinWindowSize = { 192, 136 };

        static constexpr Ui::Size kMaxWindowSize = { 600, 440 };

        enum widx
        {
            frame,
            caption,
            close_button,
            panel,
            tab_station,
            tab_cargo,
            tab_cargo_ratings,
            tab_vehicles_trains,
            tab_vehicles_buses,
            tab_vehicles_trucks,
            tab_vehicles_trams,
            tab_vehicles_aircraft,
            tab_vehicles_ships,
            content_begin,
        };

        namespace Widx
        {
            constexpr WidgetId kFrame{ "frame" };
            constexpr WidgetId kCaption{ "caption" };
            constexpr WidgetId kCloseButton{ "close_button" };
            constexpr WidgetId kPanel{ "panel" };
            constexpr WidgetId kTabStation{ "tab_station" };
            constexpr WidgetId kTabCargo{ "tab_cargo" };
            constexpr WidgetId kTabCargoRatings{ "tab_cargo_ratings" };
            constexpr WidgetId kTabVehiclesTrains{ "tab_vehicles_trains" };
            constexpr WidgetId kTabVehiclesBuses{ "tab_vehicles_buses" };
            constexpr WidgetId kTabVehiclesTrucks{ "tab_vehicles_trucks" };
            constexpr WidgetId kTabVehiclesTrams{ "tab_vehicles_trams" };
            constexpr WidgetId kTabVehiclesAircraft{ "tab_vehicles_aircraft" };
            constexpr WidgetId kTabVehiclesShips{ "tab_vehicles_ships" };
        }

        static constexpr auto makeCommonWidgets(int32_t frameWidth, int32_t frameHeight)
        {
            return makeWidgets(
                Widgets::Frame(Widx::kFrame, { 0, 0 }, { frameWidth, frameHeight }, WindowColour::primary),
                Widgets::Caption(Widx::kCaption, { 1, 1 }, { frameWidth - 2, 13 }, Widgets::Caption::Style::blackText, WindowColour::primary, StringIds::title_station),
                Widgets::ImageButton(Widx::kCloseButton, { frameWidth - 15, 2 }, { 13, 13 }, WindowColour::primary, ImageIds::close_button, StringIds::tooltip_close_window),
                Widgets::Panel(Widx::kPanel, { 0, 41 }, { frameWidth, 95 }, WindowColour::secondary),
                Widgets::Tab(Widx::kTabStation, { 3, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_station),
                Widgets::Tab(Widx::kTabCargo, { 34, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_station_cargo),
                Widgets::Tab(Widx::kTabCargoRatings, { 65, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_station_cargo_ratings),
                Widgets::Tab(Widx::kTabVehiclesTrains, { 3, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_trains),
                Widgets::Tab(Widx::kTabVehiclesBuses, { 3, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_buses),
                Widgets::Tab(Widx::kTabVehiclesTrucks, { 3, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_trucks),
                Widgets::Tab(Widx::kTabVehiclesTrams, { 3, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_trams),
                Widgets::Tab(Widx::kTabVehiclesAircraft, { 3, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_aircraft),
                Widgets::Tab(Widx::kTabVehiclesShips, { 3, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_ships));
        }

        static bool isVehicleTypeAvailable(Window& self, VehicleType vehicleType)
        {
            return (self.var_846 & (1U << enumValue(vehicleType))) != 0;
        }

        static void setVehicleTypeAvailable(Window& self, VehicleType vehicleType)
        {
            self.var_846 |= (1U << enumValue(vehicleType));
        }

        // Defined at the bottom of this file.
        static void onClose(Window& self);
        static void onMouseUp(Window& self, WidgetIndex_t widgetIndex, const WidgetId id);
        static void prepareDraw(Window& self);
        static void textInput(Window& self, WidgetIndex_t callingWidget, [[maybe_unused]] const WidgetId id, const char* input);
        static void update(Window& self);
        static void renameStationPrompt(Window* self, WidgetIndex_t widgetIndex);
        static void switchTab(Window& self, WidgetIndex_t widgetIndex);
        static void drawTabs(Window& self, Gfx::DrawingContext& drawingCtx);
        static void enableRenameByCaption(Window* self);
    }

    namespace Station
    {
        static constexpr Ui::Size kWindowSize = { 223, 136 };

        enum widx
        {
            viewport = Common::widx::content_begin,
            status_bar,
            centre_on_viewport,
        };

        namespace Widx
        {
            constexpr WidgetId kViewport{ "viewport" };
            constexpr WidgetId kStatusBar{ "status_bar" };
            constexpr WidgetId kCentreOnViewport{ "centre_on_viewport" };
        }

        static constexpr auto widgets = makeWidgets(
            // commonWidgets(kWindowSize.width, kWindowSize.height),
            Common::makeCommonWidgets(223, 136),
            Widgets::Viewport(Widx::kViewport, { 3, 44 }, { 195, 80 }, WindowColour::secondary, Widget::kContentUnk),
            Widgets::Label(Widx::kStatusBar, { 3, 115 }, { 195, 21 }, WindowColour::secondary, ContentAlign::left, StringIds::black_stringid),
            Widgets::ImageButton(Widx::kCentreOnViewport, { 0, 0 }, { 24, 24 }, WindowColour::secondary, ImageIds::centre_viewport, StringIds::move_main_view_to_show_this)

        );

        // 0x0048E352
        static void prepareDraw(Window& self)
        {
            Common::prepareDraw(self);

            self.widgets[widx::viewport].right = self.width - 4;
            self.widgets[widx::viewport].bottom = self.height - 14;

            self.widgets[widx::status_bar].top = self.height - 12;
            self.widgets[widx::status_bar].bottom = self.height - 3;
            self.widgets[widx::status_bar].right = self.width - 14;

            // Set station status
            auto station = StationManager::get(StationId(self.number));
            const char* buffer = StringManager::getString(StringIds::buffer_1250);
            station->getStatusString((char*)buffer);
            FormatArguments args{ self.widgets[widx::status_bar].textArgs };
            args.push(StringIds::buffer_1250);

            self.widgets[widx::centre_on_viewport].right = self.widgets[widx::viewport].right - 1;
            self.widgets[widx::centre_on_viewport].bottom = self.widgets[widx::viewport].bottom - 1;
            self.widgets[widx::centre_on_viewport].left = self.widgets[widx::viewport].right - 24;
            self.widgets[widx::centre_on_viewport].top = self.widgets[widx::viewport].bottom - 24;

            Widget::leftAlignTabs(self, Common::widx::tab_station, Common::widx::tab_cargo_ratings);
        }

        // 0x0048E470
        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);
        }

        // 0x0048E4D4
        static void onMouseUp(Window& self, WidgetIndex_t widgetIndex, const WidgetId id)
        {
            switch (id)
            {
                // 0x0049932D
                case Widx::kCentreOnViewport:
                    self.viewportCentreMain();
                    return;
            }

            Common::onMouseUp(self, widgetIndex, id);
        }

        static void initViewport(Window& self);

        // 0x0048E70B
        static void onResize(Window& self)
        {
            Common::enableRenameByCaption(&self);

            self.setSizeBounds(kWindowSize, Common::kMaxWindowSize);

            if (self.viewports[0] != nullptr)
            {
                uint16_t newWidth = self.width - 8;
                uint16_t newHeight = self.height - 59;

                auto& viewport = self.viewports[0];
                if (newWidth != viewport->width || newHeight != viewport->height)
                {
                    viewport->setDimensions({ newWidth, newHeight }, { newWidth, newHeight });
                    self.savedView.clear();
                }
            }

            initViewport(self);
        }

        // 0x0048F11B
        static void initViewport(Window& self)
        {
            if (self.currentTab != 0)
            {
                return;
            }

            self.callPrepareDraw();

            // Figure out the station's position on the map.
            auto station = StationManager::get(StationId(self.number));

            // Compute views.

            SavedView view = {
                station->x,
                station->y,
                ZoomLevel::half,
                static_cast<int8_t>(self.viewports[0]->getRotation()),
                station->z,
            };
            view.flags |= (1 << 14);

            ViewportFlags flags = ViewportFlags::none;
            if (self.viewports[0] != nullptr)
            {
                if (self.savedView == view)
                {
                    return;
                }

                flags = self.viewports[0]->flags;
                self.viewportRemove(0);
            }
            else
            {
                if (Config::get().gridlinesOnLandscape)
                {
                    flags |= ViewportFlags::gridlines_on_landscape;
                }
            }
            // Remove station names from viewport
            flags |= ViewportFlags::hideStationNames;

            self.savedView = view;

            // 0x0048F1CB start
            if (self.viewports[0] == nullptr)
            {
                auto widget = &self.widgets[widx::viewport];
                auto tile = World::Pos3({ station->x, station->y, station->z });
                auto origin = Ui::Point(widget->left + 1, widget->top + 1);
                auto size = Ui::Size(widget->width() - 2, widget->height() - 2);
                ViewportManager::create(&self, 0, origin, size, self.savedView.zoomLevel, tile);
                self.invalidate();
                self.flags |= WindowFlags::viewportNoScrolling;
            }
            // 0x0048F1CB end

            if (self.viewports[0] != nullptr)
            {
                self.viewports[0]->flags = flags;
                self.invalidate();
            }
        }

        static constexpr WindowEventList kEvents = {
            .onClose = Common::onClose,
            .onMouseUp = onMouseUp,
            .onResize = onResize,
            .onUpdate = Common::update,
            .textInput = Common::textInput,
            .viewportRotate = initViewport,
            .prepareDraw = prepareDraw,
            .draw = draw,
        };

        static const WindowEventList& getEvents()
        {
            return kEvents;
        }
    }

    namespace VehiclesStopping
    {
        static void populateVehicleList(Window& self);
        static void sortVehicleList(Window& self);
    }

    // 0x0048F210
    Window* open(StationId stationId)
    {
        auto window = WindowManager::bringToFront(WindowType::station, enumValue(stationId));
        if (window != nullptr)
        {
            if (ToolManager::isToolActive(window->type, window->number))
            {
                ToolManager::toolCancel();
            }

            window = WindowManager::bringToFront(WindowType::station, enumValue(stationId));
        }

        _cargoWindowStates.erase(stationId);
        if (window == nullptr)
        {
            // 0x0048F29F start
            const WindowFlags newFlags = WindowFlags::resizable | WindowFlags::lighterFrame;
            window = WindowManager::createWindow(WindowType::station, Station::kWindowSize, newFlags, Station::getEvents());
            window->number = enumValue(stationId);

            auto station = StationManager::get(stationId);
            window->owner = station->owner;

            window->setSizeBounds(Common::kMinWindowSize, Common::kMaxWindowSize);
            window->savedView.clear();

            auto skin = ObjectManager::get<InterfaceSkinObject>();
            window->setColour(WindowColour::secondary, skin->windowPlayerColor);
            // 0x0048F29F end
        }

        window->currentTab = Common::widx::tab_station - Common::widx::tab_station;
        window->invalidate();

        // We'll need the vehicle list to determine what vehicle tabs to show
        VehiclesStopping::populateVehicleList(*window);

        window->setWidgets(Station::widgets);
        window->holdableWidgets = 0;
        window->eventHandlers = &Station::getEvents();
        window->activatedWidgets = 0;
        window->disabledWidgets = 0;
        window->initScrollWidgets();
        Station::initViewport(*window);

        return window;
    }

    void reset()
    {
        _lastSelectedStation = StationId::null;
        _cargoWindowStates.clear();
    }

    namespace Cargo
    {
        static constexpr Ui::Size kMinWindowSize = { 223, 162 };

        enum widx
        {
            group_by_label = Common::widx::content_begin,
            group_by,
            group_by_dropdown,
            sort_by_label,
            sort_by,
            sort_by_dropdown,
            scrollview,
            status_bar,
            station_catchment,
        };

        namespace Widx
        {
            constexpr WidgetId kGroupByLabel{ "group_by_label" };
            constexpr WidgetId kGroupBy{ "group_by" };
            constexpr WidgetId kGroupByDropdown{ "group_by_dropdown" };
            constexpr WidgetId kSortByLabel{ "sort_by_label" };
            constexpr WidgetId kSortBy{ "sort_by" };
            constexpr WidgetId kSortByDropdown{ "sort_by_dropdown" };
            constexpr WidgetId kScrollview{ "scrollview" };
            constexpr WidgetId kStatusBar{ "status_bar" };
            constexpr WidgetId kStationCatchment{ "station_catchment" };
        }

        enum class CargoRowType : uint8_t
        {
            header,
            group,
            groupsOmitted,
        };

        struct CargoListRow
        {
            CargoRowType type{};
            int32_t y{};
            int32_t height{};
            uint8_t cargo{};
            bool expandable{};
            bool expanded{};
            CargoRouteTree::Row group{};
            size_t groupsOmitted{};
        };

        static constexpr int32_t kMaxCargoHeaderHeight = 22;
        static constexpr size_t kMaxDisplayedCargoGroups = (std::numeric_limits<int16_t>::max() - 1 - kMaxCargoStats * (kMaxCargoHeaderHeight + CargoRouteTree::kRowHeight)) / CargoRouteTree::kRowHeight;

        static CargoWindowState& getWindowState(const StationId station)
        {
            auto& state = _cargoWindowStates[station];
            const auto cargoRevision = CargoDist::getStateConst().cargoRevision;
            const auto routingRevision = CargoDist::getStateConst().routingRevision;
            if (state.cargoRevision != cargoRevision || state.routingRevision != routingRevision)
            {
                state.cargoRevision = cargoRevision;
                state.routingRevision = routingRevision;
                state.routeTrees.clear();
            }
            return state;
        }

        static std::vector<CargoDist::CargoRouteNode>& getRouteTree(CargoWindowState& state, const uint8_t cargo, const CargoDist::PacketList& packets)
        {
            auto found = state.routeTrees.find(cargo);
            if (found == state.routeTrees.end())
            {
                const auto summaries = CargoDist::getRouteSummaries(packets);
                found = state.routeTrees.emplace(cargo, CargoDist::getRouteTree(summaries, CargoRouteTree::getOrder(state.groupOrder))).first;
                CargoRouteTree::sortTree(found->second, state.sortMode);
            }
            else if (state.sortMode == CargoRouteTree::SortMode::station)
            {
                // Station and town names can change without advancing a cargo revision.
                CargoRouteTree::sortTree(found->second, state.sortMode);
            }
            return found->second;
        }

        static std::vector<CargoListRow> getRows(Window& self)
        {
            const auto stationId = StationId(self.number);
            const auto* station = StationManager::get(stationId);
            auto& state = getWindowState(stationId);
            std::vector<CargoListRow> rows;
            size_t displayedGroups = 0;
            int32_t y = 0;
            for (uint32_t cargoId = 0; cargoId < kMaxCargoStats; ++cargoId)
            {
                const auto& stats = station->cargoStats[cargoId];
                if (stats.quantity == 0)
                {
                    continue;
                }

                const auto cargo = static_cast<uint8_t>(cargoId);
                const auto* packets = CargoDist::getStationCargoConst(stationId, cargo);
                const auto expandable = packets != nullptr && !packets->empty();
                const auto expanded = expandable && (state.expanded & (1U << cargo)) != 0;
                const auto showOrigin = stats.origin != stationId && stats.origin != StationId::null;
                const auto height = 12 + (showOrigin ? 10 : 0);
                rows.push_back({
                    .type = CargoRowType::header,
                    .y = y,
                    .height = height,
                    .cargo = cargo,
                    .expandable = expandable,
                    .expanded = expanded,
                });
                y += height;

                if (expanded)
                {
                    size_t omittedGroups = 0;
                    std::vector<CargoRouteTree::Row> groupRows;
                    CargoRouteTree::appendRows(groupRows, getRouteTree(state, cargo, *packets), state.groupOrder, state.expandedGroups[cargo], kMaxDisplayedCargoGroups - displayedGroups, omittedGroups);
                    for (const auto& group : groupRows)
                    {
                        rows.push_back({
                            .type = CargoRowType::group,
                            .y = y,
                            .height = CargoRouteTree::kRowHeight,
                            .cargo = cargo,
                            .group = group,
                        });
                        y += CargoRouteTree::kRowHeight;
                    }
                    displayedGroups += groupRows.size();
                    if (omittedGroups != 0)
                    {
                        rows.push_back({
                            .type = CargoRowType::groupsOmitted,
                            .y = y,
                            .height = CargoRouteTree::kRowHeight,
                            .cargo = cargo,
                            .groupsOmitted = omittedGroups,
                        });
                        y += CargoRouteTree::kRowHeight;
                    }
                }
            }
            return rows;
        }

        static const CargoListRow* getRowAt(const std::vector<CargoListRow>& rows, const int32_t y)
        {
            const auto found = std::find_if(rows.begin(), rows.end(), [y](const auto& row) { return y >= row.y && y < row.y + row.height; });
            return found == rows.end() ? nullptr : &*found;
        }

        static constexpr auto widgets = makeWidgets(
            Common::makeCommonWidgets(kMinWindowSize.width, kMinWindowSize.height),
            Widgets::Label(Widx::kGroupByLabel, { 3, 44 }, { 40, 12 }, WindowColour::secondary, ContentAlign::right, StringIds::cargo_group_by),
            Widgets::dropdownWidgets(Widx::kGroupBy, Widx::kGroupByDropdown, { 45, 44 }, { 152, 12 }, WindowColour::secondary, StringIds::wcolour2_stringid, StringIds::tooltip_cargo_group_by),
            Widgets::Label(Widx::kSortByLabel, { 3, 57 }, { 40, 12 }, WindowColour::secondary, ContentAlign::right, StringIds::cargo_sort_by),
            Widgets::dropdownWidgets(Widx::kSortBy, Widx::kSortByDropdown, { 45, 57 }, { 152, 12 }, WindowColour::secondary, StringIds::wcolour2_stringid, StringIds::tooltip_cargo_sort_by),
            Widgets::ScrollView(Widx::kScrollview, { 3, 70 }, { 217, 78 }, WindowColour::secondary, 2),
            Widgets::Label(Widx::kStatusBar, { 3, 151 }, { 195, 10 }, WindowColour::secondary, ContentAlign::left, StringIds::accepted_cargo_separator, StringIds::small_black_string),
            Widgets::ImageButton(Widx::kStationCatchment, { 198, 44 }, { 24, 24 }, WindowColour::secondary, ImageIds::show_station_catchment, StringIds::station_catchment)

        );

        static void resizeDropdown(Window& self, const widx comboIndex, const widx buttonIndex)
        {
            auto& combo = self.widgets[comboIndex];
            combo.right = self.width - 27;
            auto& button = self.widgets[buttonIndex];
            button.right = combo.right - 1;
            button.left = button.right - 10;
        }

        // 0x0048E7C0
        static void prepareDraw(Window& self)
        {
            Common::prepareDraw(self);

            resizeDropdown(self, widx::group_by, widx::group_by_dropdown);
            resizeDropdown(self, widx::sort_by, widx::sort_by_dropdown);

            self.widgets[widx::scrollview].right = self.width - 4;
            self.widgets[widx::scrollview].bottom = self.height - 14;

            self.widgets[widx::status_bar].top = self.height - 12;
            self.widgets[widx::status_bar].bottom = self.height - 3;
            self.widgets[widx::status_bar].right = self.width - 14;

            self.widgets[widx::station_catchment].right = self.width - 2;
            self.widgets[widx::station_catchment].left = self.width - 25;

            Widget::leftAlignTabs(self, Common::widx::tab_station, Common::widx::tab_cargo_ratings);

            self.activatedWidgets &= ~(1 << widx::station_catchment);
            if (StationId(self.number) == _lastSelectedStation)
            {
                self.activatedWidgets |= (1 << widx::station_catchment);
            }

            const auto& state = getWindowState(StationId(self.number));
            {
                auto& widget = self.widgets[widx::group_by];
                FormatArguments args{ widget.textArgs };
                args.push(CargoRouteTree::getGroupOrderNames()[static_cast<size_t>(state.groupOrder)]);
            }
            {
                auto& widget = self.widgets[widx::sort_by];
                FormatArguments args{ widget.textArgs };
                args.push(CargoRouteTree::getSortModeNames()[static_cast<size_t>(state.sortMode)]);
            }
        }

        // 0x0048E8DE
        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);

            const char* acceptedLabel = StringManager::getString(StringIds::accepted_cargo_separator);
            auto labelWidth = tr.getStringWidth(acceptedLabel);
            auto origin = self.widgets[widx::status_bar].position() + Point{ labelWidth + 2, -1 };

            auto station = StationManager::get(StationId(self.number));
            uint8_t cargoTypeCount = 0;

            for (uint32_t cargoId = 0; cargoId < kMaxCargoStats; cargoId++)
            {
                auto& stats = station->cargoStats[cargoId];
                if (!stats.isAccepted())
                {
                    continue;
                }

                auto* cargoObj = ObjectManager::get<CargoObject>(cargoId);
                drawingCtx.drawImage(ZoomLevel::full, origin, cargoObj->unitInlineSprite);
                origin.x += 12;

                cargoTypeCount++;
            }

            if (cargoTypeCount == 0)
            {
                FormatArguments args{};
                args.push(StringIds::cargo_nothing_accepted);
                tr.drawStringLeft(origin + Point{ 0, 1 }, Colour::black, StringIds::black_stringid, args);
            }
        }

        // 0x0048EB0B
        static void onMouseUp(Window& self, WidgetIndex_t widgetIndex, const WidgetId id)
        {
            switch (id)
            {
                case Widx::kStationCatchment:
                {
                    StationId windowNumber = StationId(self.number);
                    if (windowNumber == _lastSelectedStation)
                    {
                        windowNumber = StationId::null;
                    }

                    showStationCatchment(windowNumber);
                    return;
                }
            }

            Common::onMouseUp(self, widgetIndex, id);
        }

        // 0x0048EBB7
        static void onResize(Window& self)
        {
            Common::enableRenameByCaption(&self);

            self.setSizeBounds(kMinWindowSize, Common::kMaxWindowSize);
        }

        // 0x0048EB64
        static void getScrollSize(Window& self, [[maybe_unused]] uint32_t scrollIndex, [[maybe_unused]] int32_t& scrollWidth, int32_t& scrollHeight)
        {
            const auto rows = getRows(self);
            scrollHeight = rows.empty() ? 0 : rows.back().y + rows.back().height;
        }

        // 0x0048EB4F
        static std::optional<FormatArguments> tooltip(Window& self, [[maybe_unused]] WidgetIndex_t widgetIndex, const WidgetId id)
        {
            FormatArguments args{};

            if (id == Widx::kScrollview)
            {
                args.push(StringIds::tooltip_scroll_cargo_list);
            }
            else if (id == Widx::kStatusBar)
            {
                // First, find out how wide the 'Accepted:' label is
                const char* acceptedLabel = StringManager::getString(StringIds::accepted_cargo_separator);
                const auto font = Gfx::Font::medium_bold;
                const int16_t labelWidth = Gfx::TextRenderer::getStringWidthNewLined(font, acceptedLabel);

                // Now find out where we're pointing relative to the label
                const auto mousePos = Input::getMouseLocation();
                const auto startPos = self.position() + self.widgets[widx::status_bar].position() + Point{ 2, -1 };
                const auto relPos = mousePos - startPos;

                // Find out which cargo icon we must be pointing at, if any
                const auto cargoPointedAt = (relPos.x - labelWidth) / 12;
                auto* station = StationManager::get(StationId(self.number));
                auto cargoIndex = 0;
                for (uint32_t cargoId = 0; cargoId < kMaxCargoStats; cargoId++)
                {
                    auto& stats = station->cargoStats[cargoId];
                    if (!stats.isAccepted())
                    {
                        continue;
                    }

                    if (cargoIndex != cargoPointedAt)
                    {
                        cargoIndex++;
                        continue;
                    }

                    auto* cargoObj = ObjectManager::get<CargoObject>(cargoId);
                    args.push(cargoObj->name);
                    return args;
                }

                return std::nullopt;
            }

            return args;
        }

        static void appendStationArguments(FormatArguments& args, const StationId stationId)
        {
            const auto* station = StationManager::get(stationId);
            args.push(station->name);
            args.push(station->town);
        }

        static void drawCargoHeader(Window& self, Gfx::DrawingContext& drawingCtx, const CargoListRow& row, const StationCargoStats& cargo)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);
            auto quantity = std::min<int32_t>(cargo.quantity, 400);
            auto units = (quantity + 9) / 10;
            const auto* cargoObj = ObjectManager::get<CargoObject>(row.cargo);
            auto xPos = static_cast<int16_t>(row.expandable ? 10 : 1);
            if (row.expandable)
            {
                CargoRouteTree::drawDisclosure(drawingCtx, 2, static_cast<int16_t>(row.y + 5), row.expanded);
            }
            while (units-- > 0)
            {
                drawingCtx.drawImage(ZoomLevel::full, xPos, static_cast<int16_t>(row.y + 1), cargoObj->unitInlineSprite);
                xPos += 10;
            }

            const auto cargoName = cargo.quantity == 1 ? cargoObj->unitNameSingular : cargoObj->unitNamePlural;
            const auto textRight = self.widgets[widx::scrollview].width() - 14;
            FormatArguments args{};
            args.push(cargoName);
            args.push<uint32_t>(cargo.quantity);
            const auto stationId = StationId(self.number);
            const auto showOrigin = cargo.origin != stationId && cargo.origin != StationId::null;
            const auto cargoString = showOrigin ? StringIds::station_cargo_en_route_start : StringIds::station_cargo;
            tr.drawStringRight({ textRight, static_cast<int16_t>(row.y + 1) }, AdvancedColour(Colour::black).outline(), cargoString, args);

            if (showOrigin)
            {
                FormatArguments originArgs{};
                appendStationArguments(originArgs, cargo.origin);
                tr.drawStringRight({ textRight, static_cast<int16_t>(row.y + 11) }, AdvancedColour(Colour::black).outline(), StringIds::station_cargo_en_route_end, originArgs);
            }
        }

        static void drawCargoGroup(Window& self, Gfx::DrawingContext& drawingCtx, const CargoListRow& row)
        {
            CargoRouteTree::drawRow(drawingCtx, row.group, row.y, self.widgets[widx::scrollview].width());
        }

        static void drawCargoGroupsOmitted(Window& self, Gfx::DrawingContext& drawingCtx, const CargoListRow& row)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);
            FormatArguments args{};
            args.push<int32_t>(static_cast<int32_t>(std::min<size_t>(row.groupsOmitted, std::numeric_limits<int32_t>::max())));
            const auto width = std::max<int32_t>(self.widgets[widx::scrollview].width() - 22, 0);
            tr.drawStringLeftClipped({ 10, static_cast<int16_t>(row.y + 1) }, width, Colour::black, StringIds::station_cargo_groups_omitted, args);
        }

        // 0x0048E986
        static void drawScroll(Window& self, Gfx::DrawingContext& drawingCtx, [[maybe_unused]] const uint32_t scrollIndex)
        {
            drawingCtx.clearSingle(Colours::getShade(self.getColour(WindowColour::secondary).c(), 4));

            const auto station = StationManager::get(StationId(self.number));
            const auto& renderTarget = drawingCtx.currentRenderTarget();
            for (const auto& row : getRows(self))
            {
                if (row.y + row.height < renderTarget.y)
                {
                    continue;
                }
                if (row.y >= renderTarget.y + renderTarget.height)
                {
                    break;
                }
                if (row.type == CargoRowType::header)
                {
                    drawCargoHeader(self, drawingCtx, row, station->cargoStats[row.cargo]);
                }
                else if (row.type == CargoRowType::group)
                {
                    drawCargoGroup(self, drawingCtx, row);
                }
                else
                {
                    drawCargoGroupsOmitted(self, drawingCtx, row);
                }
            }

            uint32_t totalUnits = 0;
            for (const auto& stats : station->cargoStats)
            {
                totalUnits += stats.quantity;
            }

            if (totalUnits == 0)
            {
                auto tr = Gfx::TextRenderer(drawingCtx);
                FormatArguments args{};
                args.push(StringIds::nothing_waiting);

                auto point = Point(1, 0);
                tr.drawStringLeft(point, Colour::black, StringIds::black_stringid, args);
            }
        }

        static void updateScrollSize(Window& self, const uint8_t scrollIndex)
        {
            self.updateScrollWidgets();
            auto& scrollArea = self.scrollAreas[scrollIndex];
            const auto visibleHeight = self.widgets[widx::scrollview].height() - 2;
            const auto maxOffset = std::max<int32_t>(scrollArea.contentHeight - visibleHeight, 0);
            scrollArea.contentOffsetY = std::clamp<int32_t>(scrollArea.contentOffsetY, 0, maxOffset);
            ScrollView::updateThumbs(self, widx::scrollview);
        }

        static void showDropdown(Window& self, const widx comboIndex, const std::span<const StringId> items, const size_t selected)
        {
            const auto& widget = self.widgets[comboIndex];
            Dropdown::show(self.x + widget.left, self.y + widget.top, widget.width() - 4, widget.height(), self.getColour(WindowColour::secondary), items.size(), 0);
            for (size_t i = 0; i < items.size(); ++i)
            {
                Dropdown::add(i, StringIds::dropdown_stringid, items[i]);
            }
            Dropdown::setItemSelected(selected);
        }

        static void onMouseDown(Window& self, [[maybe_unused]] const WidgetIndex_t widgetIndex, const WidgetId id)
        {
            const auto& state = getWindowState(StationId(self.number));
            if (id == Widx::kGroupByDropdown)
            {
                showDropdown(self, widx::group_by, CargoRouteTree::getGroupOrderNames(), static_cast<size_t>(state.groupOrder));
            }
            else if (id == Widx::kSortByDropdown)
            {
                showDropdown(self, widx::sort_by, CargoRouteTree::getSortModeNames(), static_cast<size_t>(state.sortMode));
            }
        }

        static void onDropdown(Window& self, [[maybe_unused]] const WidgetIndex_t widgetIndex, const WidgetId id, const int16_t itemIndex)
        {
            if (itemIndex < 0)
            {
                return;
            }

            auto& state = getWindowState(StationId(self.number));
            if (id == Widx::kGroupByDropdown && static_cast<size_t>(itemIndex) < CargoRouteTree::getGroupOrderNames().size())
            {
                const auto groupOrder = static_cast<CargoRouteTree::GroupOrder>(itemIndex);
                if (state.groupOrder == groupOrder)
                {
                    return;
                }
                state.groupOrder = groupOrder;
                state.expandedGroups.clear();
                state.routeTrees.clear();
            }
            else if (id == Widx::kSortByDropdown && static_cast<size_t>(itemIndex) < CargoRouteTree::getSortModeNames().size())
            {
                const auto sortMode = static_cast<CargoRouteTree::SortMode>(itemIndex);
                if (state.sortMode == sortMode)
                {
                    return;
                }
                state.sortMode = sortMode;
                state.routeTrees.clear();
            }
            else
            {
                return;
            }

            updateScrollSize(self, 0);
            self.invalidate();
        }

        static void onUpdate(Window& self)
        {
            Common::update(self);
            updateScrollSize(self, 0);
        }

        static bool isDisclosureHit(const CargoListRow& row, const int16_t x)
        {
            if (row.type == CargoRowType::header)
            {
                return row.expandable && x >= 2 && x <= 8;
            }
            return row.type == CargoRowType::group && CargoRouteTree::isDisclosureHit(row.group, x);
        }

        static bool isStationLinkHit(Window& self, const CargoListRow& row, const int16_t x)
        {
            return row.type == CargoRowType::group && CargoRouteTree::isStationLinkHit(row.group, x, self.widgets[widx::scrollview].width());
        }

        static void onScrollMouseDown(Window& self, const int16_t x, const int16_t y, const uint8_t scrollIndex)
        {
            const auto rows = getRows(self);
            const auto* row = getRowAt(rows, y);
            if (row == nullptr)
            {
                return;
            }

            if (isDisclosureHit(*row, x))
            {
                auto& state = getWindowState(StationId(self.number));
                if (row->type == CargoRowType::header)
                {
                    state.expanded ^= 1U << row->cargo;
                }
                else if (row->type == CargoRowType::group)
                {
                    auto& expandedGroups = state.expandedGroups[row->cargo];
                    if (expandedGroups.erase(row->group.key) == 0)
                    {
                        expandedGroups.insert(row->group.key);
                    }
                }
                updateScrollSize(self, scrollIndex);
                self.invalidate();
            }
            else if (isStationLinkHit(self, *row, x))
            {
                CargoRouteTree::centreOnStation(row->group.station);
            }
        }

        static CursorId cursor(Window& self, [[maybe_unused]] WidgetIndex_t widgetIndex, const WidgetId id, const int16_t x, const int16_t y, const CursorId fallback)
        {
            if (id != Widx::kScrollview)
            {
                return fallback;
            }

            const auto rows = getRows(self);
            const auto* row = getRowAt(rows, y);
            return row != nullptr && (isDisclosureHit(*row, x) || isStationLinkHit(self, *row, x)) ? CursorId::handPointer : fallback;
        }

        static constexpr WindowEventList kEvents = {
            .onClose = Common::onClose,
            .onMouseUp = onMouseUp,
            .onResize = onResize,
            .onMouseDown = onMouseDown,
            .onDropdown = onDropdown,
            .onUpdate = onUpdate,
            .getScrollSize = getScrollSize,
            .scrollMouseDown = onScrollMouseDown,
            .textInput = Common::textInput,
            .tooltip = tooltip,
            .cursor = cursor,
            .prepareDraw = prepareDraw,
            .draw = draw,
            .drawScroll = drawScroll,
        };

        static const WindowEventList& getEvents()
        {
            return kEvents;
        }
    }

    namespace CargoRatings
    {
        static constexpr Ui::Size kWindowSize = { 249, 136 };

        static constexpr Ui::Size kMaxWindowSize = { 249, 440 };

        enum widx
        {
            scrollview = Common::widx::content_begin,
            status_bar,
        };

        namespace Widx
        {
            constexpr WidgetId kScrollview{ "scrollview" };
            constexpr WidgetId kStatusBar{ "status_bar" };
        }

        static constexpr auto widgets = makeWidgets(
            Common::makeCommonWidgets(249, 136),
            Widgets::ScrollView(Widx::kScrollview, { 3, 44 }, { 244, 80 }, WindowColour::secondary, 2),
            Widgets::Label(Widx::kStatusBar, { 3, 125 }, { 221, 11 }, WindowColour::secondary, ContentAlign::center)

        );

        // 0x0048EC3B
        static void prepareDraw(Window& self)
        {
            Common::prepareDraw(self);

            self.widgets[widx::scrollview].right = self.width - 4;
            self.widgets[widx::scrollview].bottom = self.height - 14;

            self.widgets[widx::status_bar].top = self.height - 12;
            self.widgets[widx::status_bar].bottom = self.height - 3;
            self.widgets[widx::status_bar].right = self.width - 14;

            Widget::leftAlignTabs(self, Common::widx::tab_station, Common::widx::tab_cargo_ratings);
        }

        // 0x0048ED24
        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);
        }

        // 0x0048EE97
        static void onResize(Window& self)
        {
            Common::enableRenameByCaption(&self);

            self.setSizeBounds(kWindowSize, kMaxWindowSize);
        }

        // 0x0048EE4A
        static void getScrollSize(Window& self, [[maybe_unused]] uint32_t scrollIndex, [[maybe_unused]] int32_t& scrollWidth, int32_t& scrollHeight)
        {
            auto station = StationManager::get(StationId(self.number));
            scrollHeight = 0;
            for (uint8_t i = 0; i < 32; i++)
            {
                if (station->cargoStats[i].origin != StationId::null)
                {
                    scrollHeight += 10;
                }
            }
        }

        // 0x0048EE73
        static std::optional<FormatArguments> tooltip([[maybe_unused]] Ui::Window& window, [[maybe_unused]] WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id)
        {
            FormatArguments args{};
            args.push(StringIds::tooltip_scroll_ratings_list);
            return args;
        }

        // 0x0048EF02
        static void drawRatingBar(Window* self, Gfx::DrawingContext& drawingCtx, int16_t x, int16_t y, uint8_t amount, Colour colour)
        {
            drawingCtx.fillRectInset(x, y, x + 99, y + 9, self->getColour(WindowColour::secondary), Gfx::RectInsetFlags::borderInset | Gfx::RectInsetFlags::fillNone);

            uint16_t rating = (amount * 96) / 256;
            if (rating > 2)
            {
                drawingCtx.fillRectInset(x + 2, y + 2, x + 1 + rating, y + 8, colour, Gfx::RectInsetFlags::none);
            }
        }

        // 0x0048ED2F
        static void drawScroll(Window& self, Gfx::DrawingContext& drawingCtx, [[maybe_unused]] const uint32_t scrollIndex)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            drawingCtx.clearSingle(Colours::getShade(self.getColour(WindowColour::secondary).c(), 4));

            const auto station = StationManager::get(StationId(self.number));
            auto point = Point(0, 0);
            auto cargoId = 0;
            for (const auto& cargoStats : station->cargoStats)
            {
                auto& cargo = cargoStats;
                if (cargo.empty())
                {
                    cargoId++;
                    continue;
                }

                auto cargoObj = ObjectManager::get<CargoObject>(cargoId);
                point.x = 1;
                {
                    auto argsBuf = FormatArgumentsBuffer{};
                    auto args = FormatArguments{ argsBuf };
                    args.push(cargoObj->name);

                    tr.drawStringLeftClipped(point, 98, Colour::black, StringIds::wcolour2_stringid, args);
                }

                auto rating = cargo.rating;
                auto colour = Colour::green;
                if (rating < 100)
                {
                    colour = Colour::yellow;
                    if (rating < 50)
                    {
                        colour = Colour::red;
                    }
                }

                uint8_t amount = (rating * 327) / 256;
                drawRatingBar(&self, drawingCtx, 100, point.y, amount, colour);

                uint16_t percent = rating / 2;
                point.x = 201;
                {
                    auto argsBuf = FormatArgumentsBuffer{};
                    auto args = FormatArguments{ argsBuf };
                    args.push(percent);

                    tr.drawStringLeft(point, Colour::black, StringIds::station_cargo_rating_percent, args);
                }

                point.y += 10;
                cargoId++;
            }
        }

        static constexpr WindowEventList kEvents = {
            .onClose = Common::onClose,
            .onMouseUp = Common::onMouseUp,
            .onResize = onResize,
            .onUpdate = Common::update,
            .getScrollSize = getScrollSize,
            .textInput = Common::textInput,
            .tooltip = tooltip,
            .prepareDraw = prepareDraw,
            .draw = draw,
            .drawScroll = drawScroll,
        };

        static const WindowEventList& getEvents()
        {
            return kEvents;
        }
    }

    // NB: This namespace shares a fair bit of code with the VehicleList window.
    // We should look into sharing some of these functions.
    namespace VehiclesStopping
    {
        static constexpr Ui::Size kWindowSize = { 400, 200 };

        static constexpr Ui::Size kMaxWindowSize = { 600, 800 };

        enum widx
        {
            scrollview = Common::widx::content_begin,
            status_bar,
        };

        namespace Widx
        {
            constexpr WidgetId kScrollview{ "scrollview" };
            constexpr WidgetId kStatusBar{ "status_bar" };
        }

        static constexpr auto widgets = makeWidgets(
            Common::makeCommonWidgets(223, 136),
            Widgets::ScrollView(Widx::kScrollview, { 3, 44 }, { 544, 138 }, WindowColour::secondary, Scrollbars::vertical),
            Widgets::Label(Widx::kStatusBar, { 3, kWindowSize.height - 13 }, { kWindowSize.width, 10 }, WindowColour::secondary, ContentAlign::left, StringIds::black_stringid)

        );

        static bool vehicleStopsAtActiveStation(const VehicleHead* head, StationId filterStationId)
        {
            auto orders = Vehicles::OrderRingView(head->orderTableOffset);
            for (auto& order : orders)
            {
                auto* stationOrder = order.as<Vehicles::OrderStation>();
                if (stationOrder == nullptr)
                {
                    continue;
                }

                const auto stationId = stationOrder->getStation();
                if (stationId == filterStationId)
                {
                    return true;
                }
            }
            return false;
        }

        static VehicleType getCurrentVehicleType(Window& self)
        {
            return static_cast<VehicleType>(self.currentTab - (Common::widx::tab_vehicles_trains - Common::widx::tab_station));
        }

        static void populateVehicleList(Window& self)
        {
            self.rowCount = 0;

            // Populate vehicle list with relevant entity ids
            auto currentVehicleType = getCurrentVehicleType(self);
            for (auto* vehicle : VehicleManager::VehicleList())
            {
                if (!vehicleStopsAtActiveStation(vehicle, StationId(self.number)))
                {
                    continue;
                }

                Common::setVehicleTypeAvailable(self, vehicle->vehicleType);

                if (vehicle->vehicleType != currentVehicleType)
                {
                    continue;
                }

                self.rowInfo[self.rowCount++] = enumValue(vehicle->head);
            }

            sortVehicleList(self);
        }

        static bool orderByName(const VehicleHead& lhs, const VehicleHead& rhs)
        {
            char lhsString[256] = { 0 };
            {
                FormatArguments lhsArgs{};
                lhsArgs.push(lhs.ordinalNumber);
                StringManager::formatString(lhsString, lhs.name, lhsArgs);
            }

            char rhsString[256] = { 0 };
            {
                FormatArguments rhsArgs{};
                rhsArgs.push(rhs.ordinalNumber);
                StringManager::formatString(rhsString, rhs.name, rhsArgs);
            }

            return Utility::strlogicalcmp(lhsString, rhsString) < 0;
        }

        static void sortVehicleList(Window& self)
        {
            auto list = std::span<EntityId>(reinterpret_cast<EntityId*>(self.rowInfo), self.rowCount);

            std::sort(list.begin(), list.end(), [self](EntityId lhs, EntityId rhs) {
                auto* lhsVehicle = EntityManager::get<VehicleHead>(lhs);
                auto* rhsVehicle = EntityManager::get<VehicleHead>(rhs);
                return orderByName(*lhsVehicle, *rhsVehicle);
            });

            self.invalidate();
        }

        void removeTrainFromList(Window& self, EntityId head)
        {
            auto list = std::span<EntityId>(reinterpret_cast<EntityId*>(self.rowInfo), self.rowCount);

            auto newEnd = std::remove_if(list.begin(), list.end(), [head](EntityId el) { return el == head; });
            auto numRemoved = std::distance(newEnd, list.end());

            if (numRemoved > 0)
            {
                self.rowCount -= numRemoved;
            }
        }

        static void prepareDraw(Window& self)
        {
            Common::prepareDraw(self);

            static constexpr StringId kTypeToCaption[] = {
                StringIds::stringid_trains,
                StringIds::stringid_buses,
                StringIds::stringid_trucks,
                StringIds::stringid_trams,
                StringIds::stringid_aircraft,
                StringIds::stringid_ships,
            };

            auto currentVehicleType = getCurrentVehicleType(self);
            self.widgets[Common::widx::caption].text = kTypeToCaption[enumValue(currentVehicleType)];

            // Basic frame widget dimensions
            self.widgets[widx::scrollview].right = self.width - 4;
            self.widgets[widx::scrollview].bottom = self.height - 14;

            static constexpr std::pair<StringId, StringId> kTypeToFooterStringIds[]{
                { StringIds::num_trains_singular, StringIds::num_trains_plural },
                { StringIds::num_buses_singular, StringIds::num_buses_plural },
                { StringIds::num_trucks_singular, StringIds::num_trucks_plural },
                { StringIds::num_trams_singular, StringIds::num_trams_plural },
                { StringIds::num_aircrafts_singular, StringIds::num_aircrafts_plural },
                { StringIds::num_ships_singular, StringIds::num_ships_plural },
            };

            {
                // Reposition status bar
                auto& widget = self.widgets[widx::status_bar];
                widget.top = self.height - 13;
                widget.bottom = self.height - 3;

                // Set status bar
                FormatArguments args{ widget.textArgs };
                auto& footerStringPair = kTypeToFooterStringIds[enumValue(currentVehicleType)];
                args.push(self.rowCount == 1 ? footerStringPair.first : footerStringPair.second);
                args.push(self.rowCount);
            }
        }

        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);
        }

        static void drawScroll(Window& self, Gfx::DrawingContext& drawingCtx, [[maybe_unused]] const uint32_t scrollIndex)
        {
            const auto& rt = drawingCtx.currentRenderTarget();

            auto tr = Gfx::TextRenderer(drawingCtx);

            auto shade = Colours::getShade(self.getColour(WindowColour::secondary).c(), 1);
            drawingCtx.clearSingle(shade);

            auto yPos = 0;
            for (auto i = 0; i < self.rowCount; i++)
            {
                const auto vehicleId = EntityId(self.rowInfo[i]);

                // Item not in rendering context, or no vehicle available for this slot?
                if (yPos + self.rowHeight < rt.y || vehicleId == EntityId::null)
                {
                    yPos += self.rowHeight;
                    continue;
                }
                else if (yPos >= rt.y + rt.height + self.rowHeight)
                {
                    break;
                }

                auto head = EntityManager::get<VehicleHead>(vehicleId);
                if (head == nullptr)
                {
                    removeTrainFromList(self, vehicleId);
                    continue;
                }

                // Highlight selection.
                if (head->id == EntityId(self.rowHover))
                {
                    drawingCtx.drawRect(0, yPos, self.width, self.rowHeight, Colours::getShade(self.getColour(WindowColour::secondary).c(), 0), Gfx::RectFlags::none);
                }

                auto vehicle = Vehicles::Vehicle(*head);

                // Draw vehicle at the bottom of the row
                drawTrainInline(drawingCtx, vehicle, Ui::Point(0, yPos + (self.rowHeight - 28) / 2 + 6));

                // Draw vehicle status
                {
                    // Prepare status for drawing
                    auto status = head->getStatus();
                    auto args = FormatArguments::common();
                    args.push(head->name);
                    args.push(head->ordinalNumber);
                    args.push(status.status1);
                    args.push(status.status1Args);
                    args.push(status.status2);
                    args.push(status.status2Args);

                    StringId format = StringIds::vehicle_list_status_2pos;
                    if (status.status2 != StringIds::null)
                    {
                        format = StringIds::vehicle_list_status_3pos;
                    }

                    // Draw status
                    yPos += 2;
                    auto point = Point(1, yPos);
                    tr.drawStringLeftClipped(point, 308, AdvancedColour(Colour::black).outline(), format, args);
                }

                yPos += self.rowHeight - 2;
            }
        }

        static std::optional<FormatArguments> tooltip([[maybe_unused]] Window& self, [[maybe_unused]] WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id)
        {
            FormatArguments args{};
            args.push(StringIds::tooltip_scroll_vehicle_list);
            return args;
        }

        static void onUpdate(Window& self)
        {
            self.frameNo++;
            self.callPrepareDraw();

            sortVehicleList(self);
        }

        static void getScrollSize(Window& self, [[maybe_unused]] uint32_t scrollIndex, [[maybe_unused]] int32_t& scrollWidth, int32_t& scrollHeight)
        {
            scrollHeight = self.rowCount * self.rowHeight;
        }

        static CursorId cursor(Window& self, [[maybe_unused]] WidgetIndex_t widgetIdx, const WidgetId id, [[maybe_unused]] int16_t xPos, int16_t yPos, CursorId fallback)
        {
            if (id != Widx::kScrollview)
            {
                return fallback;
            }

            uint16_t currentIndex = yPos / self.rowHeight;
            if (currentIndex < self.rowCount && self.rowInfo[currentIndex] != -1)
            {
                return CursorId::handPointer;
            }

            return fallback;
        }

        static void onScrollMouseOver(Window& self, [[maybe_unused]] int16_t x, int16_t y, [[maybe_unused]] uint8_t scroll_index)
        {
            self.flags &= ~WindowFlags::notScrollView;

            uint16_t currentRow = y / self.rowHeight;
            if (currentRow < self.rowCount)
            {
                self.rowHover = self.rowInfo[currentRow];
            }
            else
            {
                self.rowHover = -1;
            }
        }

        static void onScrollMouseDown(Window& self, [[maybe_unused]] int16_t x, int16_t y, [[maybe_unused]] uint8_t scroll_index)
        {
            uint16_t currentRow = y / self.rowHeight;
            if (currentRow >= self.rowCount)
            {
                return;
            }

            EntityId currentVehicleId = EntityId(self.rowInfo[currentRow]);
            if (currentVehicleId == EntityId::null)
            {
                return;
            }

            auto* head = EntityManager::get<VehicleHead>(currentVehicleId);
            if (head == nullptr)
            {
                return;
            }

            if (head->isPlaced())
            {
                Ui::Windows::Vehicle::Main::open(head);
            }
            else
            {
                Ui::Windows::Vehicle::Details::open(head);
            }
        }

        static void onResize(Window& self)
        {
            Common::enableRenameByCaption(&self);

            self.setSizeBounds(kWindowSize, kMaxWindowSize);
        }

        static constexpr WindowEventList kEvents = {
            .onClose = Common::onClose,
            .onMouseUp = Common::onMouseUp,
            .onResize = onResize,
            .onUpdate = onUpdate,
            .onHandleInputBegin = listWindowOnHandleInputBegin,
            .onHandleInputEnd = listWindowOnHandleInputEnd,
            .getScrollSize = getScrollSize,
            .scrollMouseDown = onScrollMouseDown,
            .scrollMouseOver = onScrollMouseOver,
            .tooltip = tooltip,
            .cursor = cursor,
            .prepareDraw = prepareDraw,
            .draw = draw,
            .drawScroll = drawScroll,
        };

        static const WindowEventList& getEvents()
        {
            return kEvents;
        }
    }

    // 0x00491BC6
    void sub_491BC6()
    {
        TileLoop tileLoop;

        for (uint32_t posId = 0; posId < kMapSize; posId++)
        {
            if (isWithinCatchmentDisplay(tileLoop.current()))
            {
                TileManager::mapInvalidateTileFull(tileLoop.current());
            }
            tileLoop.next();
        }
    }

    // 0x0049271A
    void showStationCatchment(StationId stationId)
    {
        if (stationId == _lastSelectedStation)
        {
            return;
        }

        const StationId oldStationId = _lastSelectedStation;
        _lastSelectedStation = stationId;

        if (oldStationId != StationId::null)
        {
            if (World::hasMapSelectionFlag(World::MapSelectionFlags::catchmentArea))
            {
                WindowManager::invalidate(WindowType::station, enumValue(oldStationId));
                sub_491BC6();
                World::resetMapSelectionFlag(World::MapSelectionFlags::catchmentArea);
            }
        }

        const StationId newStationId = _lastSelectedStation;

        if (newStationId != StationId::null)
        {
            Ui::Windows::Construction::sub_4A6FAC();
            auto* station = StationManager::get(_lastSelectedStation);

            setCatchmentDisplay(station, CatchmentFlags::flag_0);
            World::setMapSelectionFlags(World::MapSelectionFlags::catchmentArea);

            WindowManager::invalidate(WindowType::station, enumValue(newStationId));

            sub_491BC6();
        }
    }

    namespace Common
    {
        struct TabInformation
        {
            const widx widgetIndex;
            std::span<const Widget> widgets;
            const WindowEventList& events;
            const uint8_t rowHeight;
        };

        // clang-format off
        static TabInformation tabInformationByTabOffset[] = {
            { widx::tab_station,           Station::widgets,         Station::getEvents(),          0 },
            { widx::tab_cargo,             Cargo::widgets,           Cargo::getEvents(),            0 },
            { widx::tab_cargo_ratings,     CargoRatings::widgets,    CargoRatings::getEvents(),     0 },
            { widx::tab_vehicles_trains,   VehiclesStopping::widgets, VehiclesStopping::getEvents(), 28 },
            { widx::tab_vehicles_buses,    VehiclesStopping::widgets, VehiclesStopping::getEvents(), 28 },
            { widx::tab_vehicles_trucks,   VehiclesStopping::widgets, VehiclesStopping::getEvents(), 28 },
            { widx::tab_vehicles_trams,    VehiclesStopping::widgets, VehiclesStopping::getEvents(), 28 },
            { widx::tab_vehicles_aircraft, VehiclesStopping::widgets, VehiclesStopping::getEvents(), 48 },
            { widx::tab_vehicles_ships,    VehiclesStopping::widgets, VehiclesStopping::getEvents(), 36 },
        };
        // clang-format on

        static void onClose(Window& self)
        {
            if (StationId(self.number) == _lastSelectedStation)
            {
                showStationCatchment(StationId::null);
            }
            _cargoWindowStates.erase(StationId(self.number));
        }

        static void onMouseUp(Window& self, WidgetIndex_t widgetIndex, const WidgetId id)
        {
            switch (id)
            {
                case Widx::kCaption:
                    renameStationPrompt(&self, widgetIndex);
                    break;

                case Widx::kCloseButton:
                    WindowManager::close(&self);
                    break;

                case Widx::kTabStation:
                case Widx::kTabCargo:
                case Widx::kTabCargoRatings:
                case Widx::kTabVehiclesTrains:
                case Widx::kTabVehiclesBuses:
                case Widx::kTabVehiclesTrucks:
                case Widx::kTabVehiclesTrams:
                case Widx::kTabVehiclesAircraft:
                case Widx::kTabVehiclesShips:
                    switchTab(self, widgetIndex);
                    break;
            }
        }

        // 0x0048E352, 0x0048E7C0 and 0x0048EC3B
        static void prepareDraw(Window& self)
        {
            // Hide vehicle types without known vehicles calling at this station
            for (auto i = enumValue(VehicleType::train); i <= enumValue(VehicleType::ship); i++)
            {
                if (isVehicleTypeAvailable(self, VehicleType(i)))
                {
                    self.disabledWidgets &= ~(1ULL << (widx::tab_vehicles_trains + i));
                }
                else
                {
                    self.disabledWidgets |= (1ULL << (widx::tab_vehicles_trains + i));
                }
            }

            Widget::leftAlignTabs(self, widx::tab_station, widx::tab_vehicles_ships);

            // Activate the current tab.
            self.activatedWidgets &= ~((1ULL << widx::tab_station) | (1ULL << widx::tab_cargo) | (1ULL << widx::tab_cargo_ratings) | (1ULL << widx::tab_vehicles_trains) | (1ULL << widx::tab_vehicles_buses) | (1ULL << widx::tab_vehicles_trucks) | (1ULL << widx::tab_vehicles_trams) | (1ULL << widx::tab_vehicles_aircraft) | (1ULL << widx::tab_vehicles_ships));
            widx widgetIndex = tabInformationByTabOffset[self.currentTab].widgetIndex;
            self.activatedWidgets |= (1ULL << widgetIndex);

            // Put station and town name in place.
            auto* station = StationManager::get(StationId(self.number));

            auto args = FormatArguments(self.widgets[Common::widx::caption].textArgs);
            args.push(station->name);
            args.push(station->town);
            args.push(getTransportIconsFromStationFlags(station->flags));

            // Resize common widgets.
            self.widgets[Common::widx::frame].right = self.width - 1;
            self.widgets[Common::widx::frame].bottom = self.height - 1;

            self.widgets[Common::widx::caption].right = self.width - 2;

            self.widgets[Common::widx::close_button].left = self.width - 15;
            self.widgets[Common::widx::close_button].right = self.width - 3;

            self.widgets[Common::widx::panel].right = self.width - 1;
            self.widgets[Common::widx::panel].bottom = self.height - 1;
        }

        // 0x0048E5DF
        static void textInput(Window& self, [[maybe_unused]] WidgetIndex_t callingWidget, const WidgetId id, const char* input)
        {
            if (id != Common::Widx::kCaption)
            {
                return;
            }

            GameCommands::setErrorTitle(StringIds::error_cant_rename_station);

            GameCommands::RenameStationArgs args{};

            args.stationId = StationId(self.number);
            args.nameBufferIndex = 1;
            std::memcpy(args.buffer, input, 36);

            GameCommands::doCommand(args, GameCommands::Flags::apply);

            args.nameBufferIndex = 2;

            GameCommands::doCommand(args, GameCommands::Flags::apply);

            args.nameBufferIndex = 0;

            GameCommands::doCommand(args, GameCommands::Flags::apply);
        }

        // 0x0048E6F1
        static void update(Window& self)
        {
            self.frameNo++;
            self.callPrepareDraw();
            WindowManager::invalidate(WindowType::station, self.number);
        }

        // 0x0048E5E7
        static void renameStationPrompt(Window* self, WidgetIndex_t widgetIndex)
        {
            auto station = StationManager::get(StationId(self->number));
            auto args = FormatArguments();
            args.push<int64_t>(0);
            args.push(station->name);
            args.push(station->town);

            FormatArgumentsBuffer buffer{};
            auto args2 = FormatArguments(buffer);
            args2.push(station->town);
            TextInput::openTextInput(self, StringIds::title_station_name, StringIds::prompt_type_new_station_name, station->name, widgetIndex, args2);
        }

        // 0x0048E520
        static void switchTab(Window& self, WidgetIndex_t widgetIndex)
        {
            if (self.currentTab == widx::tab_cargo - widx::tab_station && widgetIndex != widx::tab_cargo)
            {
                const auto found = _cargoWindowStates.find(StationId(self.number));
                if (found != _cargoWindowStates.end())
                {
                    found->second.routeTrees.clear();
                }
            }
            if (widgetIndex != widx::tab_cargo)
            {
                if (StationId(self.number) == _lastSelectedStation)
                {
                    showStationCatchment(StationId::null);
                }
            }

            if (ToolManager::isToolActive(self.type, self.number))
            {
                ToolManager::toolCancel();
            }

            TextInput::sub_4CE6C9(self.type, self.number);

            self.currentTab = widgetIndex - widx::tab_station;
            self.frameNo = 0;
            self.flags &= ~(WindowFlags::maximised);
            self.var_85C = -1;

            self.viewportRemove(0);

            auto tabInfo = tabInformationByTabOffset[widgetIndex - widx::tab_station];

            self.holdableWidgets = 0;
            self.eventHandlers = &tabInfo.events;
            self.activatedWidgets = 0;
            self.setWidgets(tabInfo.widgets);
            self.disabledWidgets = 0;
            self.rowHeight = tabInfo.rowHeight;

            // We'll need the vehicle list to determine what vehicle tabs to show
            VehiclesStopping::populateVehicleList(self);
            self.rowHover = -1;

            self.invalidate();
            self.callOnResize();
            self.callPrepareDraw();
            self.initScrollWidgets();
            self.invalidate();
            self.moveInsideScreenEdges();
        }

        // 0x0048EFBC
        void drawTabs(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            auto skin = ObjectManager::get<InterfaceSkinObject>();
            auto station = StationManager::get(StationId(self.number));
            auto companyColour = CompanyManager::getCompanyColour(station->owner);

            // Station tab
            {
                uint32_t imageId = Gfx::recolour(skin->img, companyColour);
                imageId += InterfaceSkin::ImageIds::toolbar_menu_stations;
                Widget::drawTab(self, drawingCtx, imageId, widx::tab_station);
            }

            // Cargo tab
            {
                static constexpr uint32_t cargoTabImageIds[] = {
                    InterfaceSkin::ImageIds::tab_cargo_delivered_frame0,
                    InterfaceSkin::ImageIds::tab_cargo_delivered_frame1,
                    InterfaceSkin::ImageIds::tab_cargo_delivered_frame2,
                    InterfaceSkin::ImageIds::tab_cargo_delivered_frame3,
                };

                uint32_t imageId = skin->img;
                if (self.currentTab == widx::tab_cargo - widx::tab_station)
                {
                    imageId += cargoTabImageIds[(self.frameNo / 8) % std::size(cargoTabImageIds)];
                }
                else
                {
                    imageId += cargoTabImageIds[0];
                }

                Widget::drawTab(self, drawingCtx, imageId, widx::tab_cargo);
            }

            // Cargo ratings tab
            {
                const uint32_t imageId = skin->img + InterfaceSkin::ImageIds::tab_cargo_ratings;
                Widget::drawTab(self, drawingCtx, imageId, widx::tab_cargo_ratings);

                auto widget = self.widgets[widx::tab_cargo_ratings];
                auto yOffset = widget.top + 14;
                auto xOffset = widget.left + 4;
                auto totalRatingBars = 0;

                for (const auto& cargoStats : station->cargoStats)
                {
                    auto& cargo = cargoStats;
                    if (!cargo.empty())
                    {
                        drawingCtx.fillRect(xOffset, yOffset, xOffset + 22, yOffset + 1, enumValue(ExtColour::unk30), Gfx::RectFlags::transparent);

                        auto ratingColour = Colour::green;
                        if (cargo.rating < 100)
                        {
                            ratingColour = Colour::yellow;
                            if (cargo.rating < 50)
                            {
                                ratingColour = Colour::red;
                            }
                        }

                        auto ratingBarLength = (cargo.rating * 30) / 256;
                        drawingCtx.fillRect(xOffset, yOffset, xOffset - 1 + ratingBarLength, yOffset + 1, Colours::getShade(ratingColour, 6), Gfx::RectFlags::none);

                        yOffset += 3;
                        totalRatingBars++;
                        if (totalRatingBars >= 4)
                        {
                            break;
                        }
                    }
                }
            }

            // clang-format off
            static constexpr std::pair<WidgetIndex_t, std::array<uint32_t, 8>> kTabAnimations[] = {
                { Common::widx::tab_vehicles_trains, {
                    InterfaceSkin::ImageIds::vehicle_train_frame_0,
                    InterfaceSkin::ImageIds::vehicle_train_frame_1,
                    InterfaceSkin::ImageIds::vehicle_train_frame_2,
                    InterfaceSkin::ImageIds::vehicle_train_frame_3,
                    InterfaceSkin::ImageIds::vehicle_train_frame_4,
                    InterfaceSkin::ImageIds::vehicle_train_frame_5,
                    InterfaceSkin::ImageIds::vehicle_train_frame_6,
                    InterfaceSkin::ImageIds::vehicle_train_frame_7,
                } },
                { Common::widx::tab_vehicles_aircraft, {
                    InterfaceSkin::ImageIds::vehicle_aircraft_frame_0,
                    InterfaceSkin::ImageIds::vehicle_aircraft_frame_1,
                    InterfaceSkin::ImageIds::vehicle_aircraft_frame_2,
                    InterfaceSkin::ImageIds::vehicle_aircraft_frame_3,
                    InterfaceSkin::ImageIds::vehicle_aircraft_frame_4,
                    InterfaceSkin::ImageIds::vehicle_aircraft_frame_5,
                    InterfaceSkin::ImageIds::vehicle_aircraft_frame_6,
                    InterfaceSkin::ImageIds::vehicle_aircraft_frame_7,
                } },
                { Common::widx::tab_vehicles_buses, {
                    InterfaceSkin::ImageIds::vehicle_buses_frame_0,
                    InterfaceSkin::ImageIds::vehicle_buses_frame_1,
                    InterfaceSkin::ImageIds::vehicle_buses_frame_2,
                    InterfaceSkin::ImageIds::vehicle_buses_frame_3,
                    InterfaceSkin::ImageIds::vehicle_buses_frame_4,
                    InterfaceSkin::ImageIds::vehicle_buses_frame_5,
                    InterfaceSkin::ImageIds::vehicle_buses_frame_6,
                    InterfaceSkin::ImageIds::vehicle_buses_frame_7,
                } },
                { Common::widx::tab_vehicles_trams, {
                    InterfaceSkin::ImageIds::vehicle_trams_frame_0,
                    InterfaceSkin::ImageIds::vehicle_trams_frame_1,
                    InterfaceSkin::ImageIds::vehicle_trams_frame_2,
                    InterfaceSkin::ImageIds::vehicle_trams_frame_3,
                    InterfaceSkin::ImageIds::vehicle_trams_frame_4,
                    InterfaceSkin::ImageIds::vehicle_trams_frame_5,
                    InterfaceSkin::ImageIds::vehicle_trams_frame_6,
                    InterfaceSkin::ImageIds::vehicle_trams_frame_7,
                } },
                { Common::widx::tab_vehicles_trucks, {
                    InterfaceSkin::ImageIds::vehicle_trucks_frame_0,
                    InterfaceSkin::ImageIds::vehicle_trucks_frame_1,
                    InterfaceSkin::ImageIds::vehicle_trucks_frame_2,
                    InterfaceSkin::ImageIds::vehicle_trucks_frame_3,
                    InterfaceSkin::ImageIds::vehicle_trucks_frame_4,
                    InterfaceSkin::ImageIds::vehicle_trucks_frame_5,
                    InterfaceSkin::ImageIds::vehicle_trucks_frame_6,
                    InterfaceSkin::ImageIds::vehicle_trucks_frame_7,
                } },
                { Common::widx::tab_vehicles_ships, {
                    InterfaceSkin::ImageIds::vehicle_ships_frame_0,
                    InterfaceSkin::ImageIds::vehicle_ships_frame_1,
                    InterfaceSkin::ImageIds::vehicle_ships_frame_2,
                    InterfaceSkin::ImageIds::vehicle_ships_frame_3,
                    InterfaceSkin::ImageIds::vehicle_ships_frame_4,
                    InterfaceSkin::ImageIds::vehicle_ships_frame_5,
                    InterfaceSkin::ImageIds::vehicle_ships_frame_6,
                    InterfaceSkin::ImageIds::vehicle_ships_frame_7,
                } },
            };
            // clang-format on

            for (auto [tab, frames] : kTabAnimations)
            {
                if (self.isDisabled(tab))
                {
                    continue;
                }

                auto isActive = tab == self.currentTab + Common::widx::tab_station;
                auto imageId = isActive ? frames[self.frameNo / 2 % 8] : frames[0];

                uint32_t image = Gfx::recolour(skin->img + imageId, companyColour);
                Widget::drawTab(self, drawingCtx, image, tab);
            }
        }

        // 0x0048E32C
        static void enableRenameByCaption(Window* self)
        {
            auto station = StationManager::get(StationId(self->number));
            if (station->owner != CompanyId::null)
            {
                if (CompanyManager::isPlayerCompany(station->owner))
                {
                    self->disabledWidgets &= ~(1 << Common::widx::caption);
                }
                else
                {
                    self->disabledWidgets |= (1 << Common::widx::caption);
                }
            }
        }
    }
}
