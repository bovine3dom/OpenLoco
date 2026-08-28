#include "Vehicles/Vehicle.h"
#include "Audio/Audio.h"
#include "Config.h"
#include "Entities/EntityManager.h"
#include "GameCommands/GameCommands.h"
#include "GameCommands/Vehicles/CloneVehicle.h"
#include "GameCommands/Vehicles/RenameVehicle.h"
#include "GameCommands/Vehicles/VehicleChangeRunningMode.h"
#include "GameCommands/Vehicles/VehicleOrderDelete.h"
#include "GameCommands/Vehicles/VehicleOrderDown.h"
#include "GameCommands/Vehicles/VehicleOrderInsert.h"
#include "GameCommands/Vehicles/VehicleOrderReverse.h"
#include "GameCommands/Vehicles/VehicleOrderShare.h"
#include "GameCommands/Vehicles/VehicleOrderSkip.h"
#include "GameCommands/Vehicles/VehicleOrderToggleUnbunching.h"
#include "GameCommands/Vehicles/VehicleOrderUp.h"
#include "GameCommands/Vehicles/VehiclePassSignal.h"
#include "GameCommands/Vehicles/VehiclePickup.h"
#include "GameCommands/Vehicles/VehiclePickupAir.h"
#include "GameCommands/Vehicles/VehiclePickupWater.h"
#include "GameCommands/Vehicles/VehiclePlace.h"
#include "GameCommands/Vehicles/VehiclePlaceAir.h"
#include "GameCommands/Vehicles/VehiclePlaceWater.h"
#include "GameCommands/Vehicles/VehicleRearrange.h"
#include "GameCommands/Vehicles/VehicleRefit.h"
#include "GameCommands/Vehicles/VehicleRepaint.h"
#include "GameCommands/Vehicles/VehicleReverse.h"
#include "GameCommands/Vehicles/VehicleSell.h"
#include "GameCommands/Vehicles/VehicleSpeedControl.h"
#include "GameCommands/Vehicles/VehicleTimetable.h"
#include "GameState.h"
#include "Graphics/Colour.h"
#include "Graphics/ImageIds.h"
#include "Graphics/RenderTarget.h"
#include "Graphics/TextRenderer.h"
#include "Input.h"
#include "LabelFrame.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/Formatting.h"
#include "Localisation/StringIds.h"
#include "Localisation/StringManager.h"
#include "Map/MapSelection.h"
#include "Map/RoadElement.h"
#include "Map/StationElement.h"
#include "Map/TileManager.h"
#include "Map/Track/SubpositionData.h"
#include "Map/Track/TrackData.h"
#include "Map/TrackElement.h"
#include "Objects/AirportObject.h"
#include "Objects/CargoObject.h"
#include "Objects/DockObject.h"
#include "Objects/InterfaceSkinObject.h"
#include "Objects/ObjectManager.h"
#include "Objects/RoadObject.h"
#include "Objects/TrackExtraObject.h"
#include "Objects/TrackObject.h"
#include "Objects/WaterObject.h"
#include "SceneManager.h"
#include "Ui/CargoRouteTree.h"
#include "Ui/Dropdown.h"
#include "Ui/ScrollView.h"
#include "Ui/ToolManager.h"
#include "Ui/ToolTip.h"
#include "Ui/ViewportInteraction.h"
#include "Ui/Widget.h"
#include "Ui/Widgets/ButtonWidget.h"
#include "Ui/Widgets/CaptionWidget.h"
#include "Ui/Widgets/ColourButtonWidget.h"
#include "Ui/Widgets/DropdownWidget.h"
#include "Ui/Widgets/FrameWidget.h"
#include "Ui/Widgets/ImageButtonWidget.h"
#include "Ui/Widgets/LabelWidget.h"
#include "Ui/Widgets/PanelWidget.h"
#include "Ui/Widgets/ScrollViewWidget.h"
#include "Ui/Widgets/SliderWidget.h"
#include "Ui/Widgets/TabWidget.h"
#include "Ui/Widgets/ViewportWidget.h"
#include "Ui/WindowManager.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/TimetableManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/Vehicle1.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBody.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleDraw.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "Vehicles/VehicleReplacement.h"
#include "Vehicles/VehicleTail.h"
#include "ViewportManager.h"
#include "World/CompanyManager.h"
#include "World/StationManager.h"

#include <OpenLoco/CargoDist/CargoDist.h>
#include <OpenLoco/Math/Trigonometry.hpp>
#include <OpenLoco/Utility/LookupTable.hpp>
#include <algorithm>
#include <charconv>
#include <fmt/core.h>
#include <limits>
#include <map>
#include <ranges>
#include <set>
#include <sfl/static_vector.hpp>
#include <span>
#include <sstream>
#include <vector>

using namespace OpenLoco::World;
using namespace OpenLoco::Literals;

using OpenLoco::GameCommands::VehicleChangeRunningModeArgs;

namespace OpenLoco::Ui::Windows::Vehicle
{
    namespace Common
    {
        enum widx
        {
            frame = 0,
            caption = 1,
            closeButton = 2,
            panel = 3,
            tabMain = 4,
            tabDetails = 5,
            tabCargo = 6,
            tabFinances = 7,
            tabRoute = 8,
        };

        namespace Widx
        {
            constexpr WidgetId kFrame{ "frame" };
            constexpr WidgetId kCaption{ "caption" };
            constexpr WidgetId kCloseButton{ "closeButton" };
            constexpr WidgetId kPanel{ "panel" };
            constexpr WidgetId kTabMain{ "tabMain" };
            constexpr WidgetId kTabDetails{ "tabDetails" };
            constexpr WidgetId kTabCargo{ "tabCargo" };
            constexpr WidgetId kTabFinances{ "tabFinances" };
            constexpr WidgetId kTabRoute{ "tabRoute" };
        }

        static constexpr auto makeCommonWidgets(int32_t frameWidth, int32_t frameHeight, StringId windowCaptionId)
        {
            return makeWidgets(
                Widgets::Frame(Widx::kFrame, { 0, 0 }, { (frameWidth), (frameHeight) }, WindowColour::primary),
                Widgets::Caption(Widx::kCaption, { 1, 1 }, { (frameWidth)-2, 13 }, Widgets::Caption::Style::colourText, WindowColour::primary, windowCaptionId),
                Widgets::ImageButton(Widx::kCloseButton, { (frameWidth)-15, 2 }, { 13, 13 }, WindowColour::primary, ImageIds::close_button, StringIds::tooltip_close_window),
                Widgets::Panel(Widx::kPanel, { 0, 41 }, { 265, 136 }, WindowColour::secondary),
                Widgets::Tab(Widx::kTabMain, { 3, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_vehicle_tab_main),
                Widgets::Tab(Widx::kTabDetails, { 34, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_vehicle_tab_details),
                Widgets::Tab(Widx::kTabCargo, { 65, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_vehicle_tab_cargo),
                Widgets::Tab(Widx::kTabFinances, { 96, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_vehicle_tab_finance),
                Widgets::Tab(Widx::kTabRoute, { 158, 15 }, { 31, 27 }, WindowColour::secondary, ImageIds::tab, StringIds::tooltip_vehicle_tab_route));
        }

        static Vehicles::VehicleHead* getVehicle(const Window& self)
        {
            auto* veh = EntityManager::get<Vehicles::VehicleHead>(EntityId(self.number));
            if (veh == nullptr)
            {
                WindowManager::close(WindowType::vehicle, self.number);
                return nullptr;
            }
            return veh;
        }

        static bool needsComponentChangeConfirm(EntityId id, const bool checkKeepCargoCheat)
        {
            auto* vehBase = EntityManager::get<Vehicles::VehicleBase>(id);
            if (vehBase == nullptr)
            {
                return false;
            }

            auto* head = EntityManager::get<Vehicles::VehicleHead>(vehBase->getHead());
            if (head == nullptr)
            {
                return false;
            }

            if (!head->hasAnyCargo())
            {
                return false;
            }

            if (checkKeepCargoCheat && Config::get().keepCargoModifyPickup)
            {
                return false;
            }

            if (head->getCarCount() > 0 && CompanyManager::getControllingId() == head->owner)
            {
                return true;
            }

            return false;
        }

        static bool confirmComponentChange(const EntityId id, const OpenLoco::StringId windowTitle, const OpenLoco::StringId windowMessage, const OpenLoco::StringId windowConfirm, const bool checkKeepCargoCheat = true)
        {
            if (!needsComponentChangeConfirm(id, checkKeepCargoCheat))
            {
                return true;
            }

            auto format = FormatArguments{};
            return Windows::PromptOkCancel::open(windowTitle, windowMessage, format, windowConfirm);
        }

        static bool confirmComponentChange(const EntityId srcId, const EntityId destId, const OpenLoco::StringId windowTitle, const OpenLoco::StringId windowMessage, const OpenLoco::StringId windowConfirm, const bool checkKeepCargoCheat = true)
        {
            if (!needsComponentChangeConfirm(srcId, checkKeepCargoCheat) && !needsComponentChangeConfirm(destId, checkKeepCargoCheat))
            {
                return true;
            }

            auto format = FormatArguments{};
            return Windows::PromptOkCancel::open(windowTitle, windowMessage, format, windowConfirm);
        }

        static void onClose(Window& self);
        static void setActiveTabs(Window& self);
        static void textInput(Window& self, const WidgetIndex_t callingWidget, const WidgetId id, const char* const input);
        static void renameVehicle(Window& self, const WidgetIndex_t widgetIndex);
        static void switchTab(Window& self, const WidgetIndex_t widgetIndex);
        static void setCaptionEnableState(Window& self);
        static void onPickup(Window& self, const WidgetIndex_t pickupWidx);
        static size_t getNumCars(Ui::Window& self);
        static void drawTabs(Window& window, Gfx::DrawingContext& drawingCtx);
        static void pickupToolUpdate(Window& self, const int16_t x, const int16_t y);
        static void pickupToolDown(Window& self, const int16_t x, const int16_t y);
        static void pickupToolAbort(Window& self);
        static size_t getNumCars(Ui::Window& self);
        static std::optional<Vehicles::Car> getCarFromScrollView(Window& self, const int16_t y);
        static std::pair<uint32_t, StringId> getPickupImageIdandTooltip(const Vehicles::VehicleHead& head, const bool isPlaced);
    }

    namespace Details
    {
        static constexpr Ui::Size kMinWindowSize = { 192, 166 };
        static constexpr Ui::Size kMinWindowSizeWithPaintEnabled = { 192, 182 };
        static constexpr Ui::Size kMaxWindowSize = { 400, 440 };

        enum widx
        {
            buildNew = Common::widx::tabRoute + 1,
            pickup,
            remove,
            paintBrush,
            paintColourPrimary,
            paintColourSecondary,
            carList,
        };

        namespace Widx
        {
            constexpr WidgetId kBuildNew{ "buildNew" };
            constexpr WidgetId kPickup{ "pickup" };
            constexpr WidgetId kRemove{ "remove" };
            constexpr WidgetId kPaintBrush{ "paintBrush" };
            constexpr WidgetId kPaintColourPrimary{ "paintColourPrimary" };
            constexpr WidgetId kPaintColourSecondary{ "paintColourSecondary" };
            constexpr WidgetId kCarList{ "carList" };
        }

        struct BodyItem
        {
            uint32_t image;
            int32_t dist;
            EntityId body;
        };

        struct BodyItems
        {
            sfl::static_vector<BodyItem, VehicleObject::kMaxBodySprites * (2 + 1)> items;
            int32_t totalDistance;
        };

        constexpr uint64_t holdableWidgets = 0;

        static constexpr auto widgets = makeWidgets(
            Common::makeCommonWidgets(265, 177, StringIds::title_vehicle_details),
            Widgets::ImageButton(Widx::kBuildNew, { 240, 44 }, { 24, 24 }, WindowColour::secondary, Widget::kContentNull, StringIds::tooltip_build_new_vehicle_for),
            Widgets::ImageButton(Widx::kPickup, { 240, 68 }, { 24, 24 }, WindowColour::secondary, Widget::kContentNull, StringIds::tooltip_remove_from_track),
            Widgets::ImageButton(Widx::kRemove, { 240, 96 }, { 24, 24 }, WindowColour::secondary, ImageIds::rubbish_bin, StringIds::tooltip_sell_or_drag_vehicle),
            Widgets::ImageButton(Widx::kPaintBrush, { 240, 122 }, { 24, 24 }, WindowColour::secondary, ImageIds::paintbrush, StringIds::vehicleRepaintTooltip),
            Widgets::ColourButton(Widx::kPaintColourPrimary, { 240, 150 }, { 16, 16 }, WindowColour::secondary, Widget::kContentNull, StringIds::tooltip_select_main_colour),
            Widgets::ColourButton(Widx::kPaintColourSecondary, { 258, 150 }, { 16, 16 }, WindowColour::secondary, Widget::kContentNull, StringIds::tooltip_select_secondary_colour),
            Widgets::ScrollView(Widx::kCarList, { 3, 44 }, { 237, 110 }, WindowColour::secondary, Scrollbars::vertical)

        );

        static void paintEntireTrain(Window& self);
        static void paintToolBegin(Window& self);
        static void paintToolDown(Window& self, const int16_t x, const int16_t y);
        static void paintToolAbort(Window& self);
        static void paintToolDownScroll(Window& self, Vehicles::Car car, const int16_t x);
        static bool isPaintToolActive(Window& self);
        static ColourScheme getPaintToolColour(Window& self);
        static BodyItems getBodyItemsForVehicle(const VehicleObject& vehObject, const uint8_t yaw, const Vehicles::Car& car);
    }

    namespace Cargo
    {
        static constexpr Ui::Size kMinWindowSize = { 192, 168 };
        static constexpr Ui::Size kMaxWindowSize = { 400, 440 };

        enum widx
        {
            refit = 9,
            expandAllButton,
            groupByLabel,
            groupBy,
            groupByDropdown,
            sortByLabel,
            sortBy,
            sortByDropdown,
            cargoList,
        };

        namespace Widx
        {
            constexpr WidgetId kRefit{ "refit" };
            constexpr WidgetId kExpandAll{ "expandAll" };
            constexpr WidgetId kGroupByLabel{ "groupByLabel" };
            constexpr WidgetId kGroupBy{ "groupBy" };
            constexpr WidgetId kGroupByDropdown{ "groupByDropdown" };
            constexpr WidgetId kSortByLabel{ "sortByLabel" };
            constexpr WidgetId kSortBy{ "sortBy" };
            constexpr WidgetId kSortByDropdown{ "sortByDropdown" };
            constexpr WidgetId kCargoList{ "cargoList" };
        }

        constexpr uint64_t holdableWidgets = 0;

        static constexpr auto widgets = makeWidgets(
            Common::makeCommonWidgets(265, 177, StringIds::title_vehicle_cargo),
            Widgets::ImageButton(Widx::kRefit, { 240, 44 }, { 24, 24 }, WindowColour::secondary, ImageIds::refit_cargo_button, StringIds::refit_vehicle_tip),
            Widgets::Button(Widx::kExpandAll, { 3, 44 }, { 40, 12 }, WindowColour::secondary, StringIds::select_all),
            Widgets::Label(Widx::kGroupByLabel, { 46, 44 }, { 40, 12 }, WindowColour::secondary, ContentAlign::right, StringIds::cargo_group_by),
            Widgets::dropdownWidgets(Widx::kGroupBy, Widx::kGroupByDropdown, { 87, 44 }, { 151, 12 }, WindowColour::secondary, StringIds::wcolour2_stringid, StringIds::tooltip_cargo_group_by),
            Widgets::Label(Widx::kSortByLabel, { 46, 57 }, { 40, 12 }, WindowColour::secondary, ContentAlign::right, StringIds::cargo_sort_by),
            Widgets::dropdownWidgets(Widx::kSortBy, Widx::kSortByDropdown, { 87, 57 }, { 151, 12 }, WindowColour::secondary, StringIds::wcolour2_stringid, StringIds::tooltip_cargo_sort_by),
            Widgets::ScrollView(Widx::kCargoList, { 3, 70 }, { 259, 94 }, WindowColour::secondary, Scrollbars::vertical)

        );

        static void eraseWindowState(EntityId vehicle);
    }

    namespace Finances
    {
        static constexpr Ui::Size kMinWindowSize = { 400, 202 };
        static constexpr Ui::Size kMaxWindowSize = kMinWindowSize;

        constexpr uint64_t holdableWidgets = 0;

        // 0x00522470
        static constexpr auto widgets = makeWidgets(
            Common::makeCommonWidgets(636, 319, StringIds::title_company_finances)

        );
    }

    namespace Route
    {
        static constexpr Ui::Size kMinWindowSize = { 265, 300 };
        static constexpr Ui::Size kMaxWindowSize = { 600, 440 };

        enum widx
        {
            tool = Common::widx::tabRoute + 1, // Only used to hold the tool does nothing
            localMode,
            expressMode,
            orderUnbunch,
            sharedOrderStatus,
            sharedOrderPrimary,
            sharedOrderLeave,
            sharedOrderReplace,
            routeList,
            orderForceUnload,
            orderWait,
            orderSkip,
            orderDelete,
            orderUp,
            orderDown,
            orderReverse,
            timetableToggle,
            timetableTravel,
            timetableDwell,
            timetablePeriod,
            timetablePhase,
            timetableMaxDelay,
            timetableSlots,
            timetableAddSlot,
            timetableClockRate,
            timetableClearDispatch,
            timetableResetDispatch,
        };

        namespace Widx
        {
            constexpr WidgetId kLocalMode{ "localMode" };
            constexpr WidgetId kExpressMode{ "expressMode" };
            constexpr WidgetId kOrderUnbunch{ "orderUnbunch" };
            constexpr WidgetId kSharedOrderStatus{ "sharedOrderStatus" };
            constexpr WidgetId kSharedOrderPrimary{ "sharedOrderPrimary" };
            constexpr WidgetId kSharedOrderLeave{ "sharedOrderLeave" };
            constexpr WidgetId kSharedOrderReplace{ "sharedOrderReplace" };
            constexpr WidgetId kRouteList{ "routeList" };
            constexpr WidgetId kOrderForceUnload{ "orderForceUnload" };
            constexpr WidgetId kOrderWait{ "orderWait" };
            constexpr WidgetId kOrderSkip{ "orderSkip" };
            constexpr WidgetId kOrderDelete{ "orderDelete" };
            constexpr WidgetId kOrderUp{ "orderUp" };
            constexpr WidgetId kOrderDown{ "orderDown" };
            constexpr WidgetId kOrderReverse{ "orderReverse" };
            constexpr WidgetId kTimetableToggle{ "timetableToggle" };
            constexpr WidgetId kTimetableTravel{ "timetableTravel" };
            constexpr WidgetId kTimetableDwell{ "timetableDwell" };
            constexpr WidgetId kTimetablePeriod{ "timetablePeriod" };
            constexpr WidgetId kTimetablePhase{ "timetablePhase" };
            constexpr WidgetId kTimetableMaxDelay{ "timetableMaxDelay" };
            constexpr WidgetId kTimetableSlots{ "timetableSlots" };
            constexpr WidgetId kTimetableAddSlot{ "timetableAddSlot" };
            constexpr WidgetId kTimetableClockRate{ "timetableClockRate" };
            constexpr WidgetId kTimetableClearDispatch{ "timetableClearDispatch" };
            constexpr WidgetId kTimetableResetDispatch{ "timetableResetDispatch" };
        }

        constexpr uint64_t holdableWidgets = 0;
        constexpr auto lineHeight = 10;

        static constexpr auto widgets = makeWidgets(
            Common::makeCommonWidgets(265, 300, StringIds::title_vehicle_route),
            // TODO: This is not ideal, this is used for the tool, do this in a better way.
            makeWidget({ 0, 0 }, { 1, 1 }, WidgetType::empty, WindowColour::primary),
            Widgets::Button(Widx::kLocalMode, { 3, 44 }, { 118, 12 }, WindowColour::secondary, StringIds::local_mode_button),
            Widgets::Button(Widx::kExpressMode, { 121, 44 }, { 119, 12 }, WindowColour::secondary, StringIds::express_mode_button),
            Widgets::Button(Widx::kOrderUnbunch, { 3, 56 }, { 237, 12 }, WindowColour::secondary, StringIds::unbunching_button, StringIds::tooltip_route_toggle_unbunching),
            Widgets::Label(Widx::kSharedOrderStatus, { 3, 70 }, { 100, 12 }, WindowColour::secondary, ContentAlign::left),
            Widgets::Button(Widx::kSharedOrderPrimary, { 103, 70 }, { 95, 12 }, WindowColour::secondary, StringIds::use_shared_orders_from, StringIds::tooltip_use_shared_orders_from),
            Widgets::Button(Widx::kSharedOrderLeave, { 198, 70 }, { 42, 12 }, WindowColour::secondary, StringIds::leave_shared_orders, StringIds::tooltip_leave_shared_orders),
            Widgets::Button(Widx::kSharedOrderReplace, { 3, 82 }, { 237, 12 }, WindowColour::secondary, StringIds::replace_shared_order_vehicles, StringIds::tooltip_replace_shared_order_vehicles),
            Widgets::ScrollView(Widx::kRouteList, { 3, 96 }, { 237, 82 }, WindowColour::secondary, Scrollbars::vertical, StringIds::tooltip_route_scrollview),
            Widgets::ImageButton(Widx::kOrderForceUnload, { 240, 44 }, { 24, 24 }, WindowColour::secondary, ImageIds::route_force_unload, StringIds::tooltip_route_insert_force_unload),
            Widgets::ImageButton(Widx::kOrderWait, { 240, 68 }, { 24, 24 }, WindowColour::secondary, ImageIds::route_wait, StringIds::tooltip_route_insert_wait_full_cargo),
            Widgets::ImageButton(Widx::kOrderSkip, { 240, 92 }, { 24, 24 }, WindowColour::secondary, ImageIds::route_skip, StringIds::tooltip_route_skip_next_order),
            Widgets::ImageButton(Widx::kOrderDelete, { 240, 116 }, { 24, 24 }, WindowColour::secondary, ImageIds::route_delete, StringIds::tooltip_route_delete_order),
            Widgets::ImageButton(Widx::kOrderUp, { 240, 140 }, { 24, 12 }, WindowColour::secondary, ImageIds::red_arrow_up, StringIds::tooltip_route_move_order_up),
            Widgets::ImageButton(Widx::kOrderDown, { 240, 152 }, { 24, 12 }, WindowColour::secondary, ImageIds::red_arrow_down, StringIds::tooltip_route_move_order_down),
            Widgets::ImageButton(Widx::kOrderReverse, { 240, 164 }, { 24, 24 }, WindowColour::secondary, ImageIds::construction_right_turnaround, StringIds::reverseOrderTableTooltip),
            Widgets::Button(Widx::kTimetableToggle, { 3, 94 }, { 237, 12 }, WindowColour::secondary, StringIds::timetable_disabled, StringIds::tooltip_timetable_toggle),
            Widgets::Button(Widx::kTimetableTravel, { 3, 218 }, { 118, 12 }, WindowColour::secondary, StringIds::timetable_value_unset, StringIds::tooltip_timetable_minutes),
            Widgets::Button(Widx::kTimetableDwell, { 121, 218 }, { 119, 12 }, WindowColour::secondary, StringIds::timetable_value_unset, StringIds::tooltip_timetable_minutes),
            Widgets::Button(Widx::kTimetablePeriod, { 3, 231 }, { 78, 12 }, WindowColour::secondary, StringIds::timetable_value_unset, StringIds::tooltip_timetable_minutes),
            Widgets::Button(Widx::kTimetablePhase, { 81, 231 }, { 78, 12 }, WindowColour::secondary, StringIds::timetable_value_unset, StringIds::tooltip_timetable_minutes),
            Widgets::Button(Widx::kTimetableMaxDelay, { 159, 231 }, { 81, 12 }, WindowColour::secondary, StringIds::timetable_value_unset, StringIds::tooltip_timetable_minutes),
            Widgets::Button(Widx::kTimetableSlots, { 3, 244 }, { 156, 12 }, WindowColour::secondary, StringIds::timetable_slots_count, StringIds::tooltip_timetable_slots),
            Widgets::Button(Widx::kTimetableAddSlot, { 159, 244 }, { 81, 12 }, WindowColour::secondary, StringIds::timetable_add_slot, StringIds::tooltip_timetable_add_slot),
            Widgets::Button(Widx::kTimetableClockRate, { 3, 257 }, { 78, 12 }, WindowColour::secondary, StringIds::timetable_value_minutes, StringIds::tooltip_timetable_clock_rate),
            Widgets::Button(Widx::kTimetableClearDispatch, { 81, 257 }, { 78, 12 }, WindowColour::secondary, StringIds::timetable_clear_dispatch, StringIds::tooltip_timetable_clear_dispatch),
            Widgets::Button(Widx::kTimetableResetDispatch, { 159, 257 }, { 81, 12 }, WindowColour::secondary, StringIds::timetable_reset_dispatch, StringIds::tooltip_timetable_reset_dispatch)

        );
    }

    static int32_t _ghostTrackProgress;          // 0x01136264
    static uint8_t _ghostAirportNode;            // 0x01136264
    static World::Pos3 _ghostVehiclePos;         // 0x0113625E
    static StationId _ghostAirportStationId;     // 0x0113625A
    static uint32_t _ghostLandTrackAndDirection; // 0x0113625A

    namespace Main
    {
        static constexpr Ui::Size kWindowSize = { 265, 177 };
        static constexpr Ui::Size kMinWindowSize = { 192, 177 };
        static constexpr Ui::Size kMaxWindowSize = { 600, 440 };

        enum widx
        {
            viewport = Common::widx::tabRoute + 1,
            status = 10,
            speedControl = 11,
            stopStart = 12,
            pickup = 13,
            passSignal = 14,
            changeDirection = 15,
            centreViewport = 16,
        };

        namespace Widx
        {
            constexpr WidgetId kViewport{ "viewport" };
            constexpr WidgetId kStatus{ "status" };
            constexpr WidgetId kSpeedControl{ "speedControl" };
            constexpr WidgetId kStopStart{ "stopStart" };
            constexpr WidgetId kPickup{ "pickup" };
            constexpr WidgetId kPassSignal{ "passSignal" };
            constexpr WidgetId kChangeDirection{ "changeDirection" };
            constexpr WidgetId kCentreViewport{ "centreViewport" };
        }

        static constexpr auto widgets = makeWidgets(
            Common::makeCommonWidgets(265, 177, StringIds::stringid),
            Widgets::Viewport(Widx::kViewport, { 3, 44 }, { 237, 120 }, WindowColour::secondary),
            Widgets::Label(Widx::kStatus, { 3, 155 }, { 237, 21 }, WindowColour::secondary, ContentAlign::center),
            Widgets::Slider(Widx::kSpeedControl, { 240, 46 }, { 24, 115 }, WindowColour::secondary),
            Widgets::ImageButton(Widx::kStopStart, { 240, 44 }, { 24, 24 }, WindowColour::secondary, ImageIds::red_flag, StringIds::tooltip_stop_start),
            Widgets::ImageButton(Widx::kPickup, { 240, 68 }, { 24, 24 }, WindowColour::secondary, ImageIds::null, StringIds::tooltip_remove_from_track),
            Widgets::ImageButton(Widx::kPassSignal, { 240, 92 }, { 24, 24 }, WindowColour::secondary, ImageIds::pass_signal, StringIds::tooltip_pass_signal_at_danger),
            Widgets::ImageButton(Widx::kChangeDirection, { 240, 116 }, { 24, 24 }, WindowColour::secondary, ImageIds::construction_right_turnaround, StringIds::tooltip_change_direction),
            Widgets::ImageButton(Widx::kCentreViewport, { 0, 0 }, { 24, 24 }, WindowColour::secondary, ImageIds::centre_viewport, StringIds::move_main_view_to_show_this)

        );

        constexpr uint64_t interactiveWidgets = (1 << widx::stopStart) | (1 << widx::pickup) | (1 << widx::passSignal) | (1 << widx::changeDirection) | (1 << widx::centreViewport);
        constexpr uint64_t holdableWidgets = 1 << widx::speedControl;

        // 0x004B5D82
        static void resetDisabledWidgets(Window& self)
        {
            self.disabledWidgets = 0;
        }

        // 0x004B5D88
        // 0x004B32F9
        static void createViewport(Window& self)
        {
            if (self.currentTab != (Common::widx::tabMain - Common::widx::tabMain))
            {
                return;
            }

            self.callPrepareDraw();

            auto vehHead = Common::getVehicle(self);
            if (vehHead == nullptr)
            {
                return;
            }
            Vehicles::Vehicle train(*vehHead);

            // If picked up no need for viewport drawn
            if (vehHead->tileX == -1)
            {
                self.viewportRemove(0);
                self.invalidate();
                return;
            }

            // By default focus on the veh2 id and if there are cars focus on the body of the first car
            EntityId targetEntity = train.veh2->id;
            if (!train.cars.empty())
            {
                targetEntity = train.cars.firstCar.front->id;
                // Always true so above is pointless
                if (train.cars.firstCar.front->getSubType() == Vehicles::VehicleEntityType::bogie)
                {
                    targetEntity = train.cars.firstCar.body->id;
                }
            }

            // Compute views.
            SavedView view = {
                targetEntity,
                (1 << 15) | (1 << 14),
                ZoomLevel::full,
                static_cast<int8_t>(self.viewports[0]->getRotation()),
                0
            };

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

            self.savedView = view;

            // 0x004B5E88 start
            if (self.viewports[0] == nullptr)
            {
                auto widget = &self.widgets[widx::viewport];
                auto origin = Ui::Point(widget->left + 1, widget->top + 1);
                auto size = Ui::Size(widget->width() - 2, widget->height() - 2);
                ViewportManager::create(&self, 0, origin, size, self.savedView.zoomLevel, targetEntity);
                self.invalidate();
                self.flags |= WindowFlags::viewportNoScrolling;
            }
            // 0x004B5E88 end

            if (self.viewports[0] != nullptr)
            {
                self.viewports[0]->flags = flags;
                self.invalidate();
            }
        }

        static constexpr uint16_t rowHeights[vehicleTypeCount] = {
            22,
            22,
            22,
            22,
            82,
            45
        };

        static const WindowEventList& getEvents();

        // 0x004B60DC
        static Window* create(const EntityId head)
        {
            auto* const self = WindowManager::createWindow(WindowType::vehicle, kWindowSize, WindowFlags::lighterFrame | WindowFlags::viewportNoShiftPixels | WindowFlags::resizable, Main::getEvents());
            self->setWidgets(widgets);
            self->number = enumValue(head);
            const auto* vehicle = Common::getVehicle(*self);
            if (vehicle == nullptr)
            {
                return self;
            }
            self->owner = vehicle->owner;
            self->rowHeight = rowHeights[static_cast<uint8_t>(vehicle->vehicleType)];
            self->currentTab = 0;
            self->frameNo = 0;
            resetDisabledWidgets(*self);
            self->minWidth = kMinWindowSize.width;
            self->minHeight = kMinWindowSize.height;
            self->maxWidth = kMaxWindowSize.width;
            self->maxHeight = kMaxWindowSize.height;
            self->var_85C = -1;
            WindowManager::close(WindowType::dragVehiclePart, 0);

            const auto* skin = ObjectManager::get<InterfaceSkinObject>();
            self->setColour(WindowColour::secondary, skin->windowPlayerColor);
            return self;
        }

        // 0x004B6033
        Window* open(const Vehicles::VehicleBase* vehicle)
        {
            if (vehicle == nullptr)
            {
                return nullptr;
            }
            const auto head = vehicle->getHead();
            auto* self = WindowManager::find(WindowType::vehicle, enumValue(head));
            if (self != nullptr)
            {
                if (ToolManager::isToolActive(self->type, self->number))
                {
                    ToolManager::toolCancel();
                }
                self = WindowManager::bringToFront(WindowType::vehicle, enumValue(head));
                Common::switchTab(*self, Common::widx::tabMain);
                return self;
            }
            self = create(head);
            self->savedView.clear();
            self->currentTab = 0;
            self->invalidate();
            self->setWidgets(widgets);
            self->holdableWidgets = holdableWidgets;
            self->eventHandlers = &getEvents();
            self->activatedWidgets = 0;
            resetDisabledWidgets(*self);
            self->initScrollWidgets();
            createViewport(*self);
            return self;
        }

        // 0x004B288F
        static void onChangeDirection(Window& self)
        {
            if (ToolManager::isToolActive(self.type, self.number, widx::pickup))
            {
                getGameState().pickupDirection = getGameState().pickupDirection ^ 1;
                return;
            }
            GameCommands::setErrorTitle(StringIds::cant_reverse_train);
            GameCommands::VehicleReverseArgs args{};
            args.head = static_cast<EntityId>(self.number);
            GameCommands::doCommand(args, GameCommands::Flags::apply);
        }

        static void onCentreViewportControl(Window& self)
        {
            Dropdown::add(0, StringIds::dropdown_stringid, StringIds::dropdown_viewport_move);
            Dropdown::add(1, StringIds::dropdown_stringid, StringIds::dropdown_viewport_focus);

            Widget& widget = self.widgets[widx::centreViewport];
            Dropdown::showText(
                self.x + widget.left,
                self.y + widget.top,
                widget.width(),
                widget.height(),
                self.getColour(WindowColour::secondary),
                2,
                0);

            Dropdown::setItemSelected(0);
            Dropdown::setHighlightedItem(0);
        }

        // 0x004B24D1
        static void onMouseUp(Window& self, const WidgetIndex_t widgetIndex, const WidgetId id)
        {
            switch (id)
            {
                case Common::Widx::kCloseButton:
                    WindowManager::close(&self);
                    break;
                case Common::Widx::kCaption:
                    Common::renameVehicle(self, widgetIndex);
                    break;
                case Common::Widx::kTabMain:
                case Common::Widx::kTabDetails:
                case Common::Widx::kTabCargo:
                case Common::Widx::kTabFinances:
                case Common::Widx::kTabRoute:
                    Common::switchTab(self, widgetIndex);
                    break;
                case Widx::kPickup:
                    Common::onPickup(self, widx::pickup);
                    break;
                case Widx::kChangeDirection:
                    onChangeDirection(self);
                    break;
                case Widx::kPassSignal:
                {
                    GameCommands::VehiclePassSignalArgs args{};
                    args.head = EntityId(self.number);

                    GameCommands::setErrorTitle(StringIds::cant_pass_signal_at_danger);
                    GameCommands::doCommand(args, GameCommands::Flags::apply);
                    break;
                }
            }
        }

        // 0x004B30F3
        static void onUpdate(Window& self)
        {
            self.frameNo += 1;
            self.callPrepareDraw();

            WindowManager::invalidateWidget(WindowType::vehicle, self.number, Common::widx::tabMain);
            WindowManager::invalidateWidget(WindowType::vehicle, self.number, widx::status);
            WindowManager::invalidateWidget(WindowType::vehicle, self.number, widx::pickup);
            WindowManager::invalidateWidget(WindowType::vehicle, self.number, widx::passSignal);
            WindowManager::invalidateWidget(WindowType::vehicle, self.number, widx::changeDirection);

            if (self.isDisabled(widx::pickup))
            {
                ToolManager::toolCancel(WindowType::vehicle, self.number);
                return;
            }

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            // If vehicle not placed put into pick-up mode if window in focus
            if (head->isPlaced())
            {
                return;
            }

            if (!WindowManager::isInFront(&self))
            {
                return;
            }

            if (head->owner != CompanyManager::getControllingId())
            {
                return;
            }

            if (!ToolManager::isToolActive(WindowType::vehicle, self.number))
            {
                Common::onPickup(self, widx::pickup);
            }
        }

        // 0x004B3210
        static void onResize(Window& self)
        {
            if (self.viewports[0] != nullptr)
            {
                auto head = Common::getVehicle(self);
                if (head == nullptr)
                {
                    return;
                }
                uint16_t newWidth = self.width - 30;
                if (head->owner != CompanyManager::getControllingId())
                {
                    newWidth += 22;
                }

                uint16_t newHeight = self.height - 59;
                if (head->hasVehicleFlags(Vehicles::VehicleFlags::manualControl) && head->owner == CompanyManager::getControllingId())
                {
                    newWidth -= 27;
                }

                auto& viewport = self.viewports[0];
                if (newWidth != viewport->width || newHeight != viewport->height)
                {
                    self.invalidate();
                    viewport->setDimensions({ newWidth, newHeight }, { newWidth, newHeight });
                    self.savedView.clear();
                }
            }
            createViewport(self);
        }

        // 0x004B274B
        static void stopStartOpen(Window& self)
        {
            Dropdown::add(0, StringIds::dropdown_stringid, StringIds::stop);
            Dropdown::add(1, StringIds::dropdown_stringid, StringIds::start);
            Dropdown::add(2, StringIds::dropdown_stringid, StringIds::manual);

            auto dropdownCount = 2;
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            if (head->mode == TransportMode::rail && SceneManager::isDriverCheatEnabled())
            {
                dropdownCount = 3;
            }

            Widget& widget = self.widgets[widx::stopStart];
            Dropdown::showText(
                self.x + widget.left,
                self.y + widget.top,
                widget.width(),
                widget.height(),
                self.getColour(WindowColour::secondary),
                dropdownCount,
                0);

            auto selected = 0; // Stop
            if (!head->hasVehicleFlags(Vehicles::VehicleFlags::commandStop))
            {
                selected = 1; // Start
            }
            if (head->hasVehicleFlags(Vehicles::VehicleFlags::manualControl))
            {
                selected = 2; // Manual
            }

            Dropdown::setItemSelected(selected);
            Dropdown::setHighlightedItem(selected == 0 ? 1 : 0); // Stop becomes start highlighted. Manual or Start becomes Stop highlighted
        }

        // 0x004B2637
        static void onSpeedControl(Window& self)
        {
            Input::setClickRepeatTicks(31);
            auto pos = Input::getScrollLastLocation();
            auto speed = pos.y - (self.y + self.widgets[widx::speedControl].top + 58);
            speed = -(std::clamp(speed, -40, 40));

            GameCommands::VehicleSpeedControlArgs args{};
            args.head = EntityId(self.number);
            args.speed = speed;
            GameCommands::doCommand(args, GameCommands::Flags::apply);
        }

        // 0x004B251A
        static void onMouseDown(Window& self, [[maybe_unused]] const WidgetIndex_t widgetIndex, const WidgetId id)
        {
            switch (id)
            {
                case Widx::kStopStart:
                    stopStartOpen(self);
                    break;
                case Widx::kSpeedControl:
                    onSpeedControl(self);
                    break;
                case Widx::kCentreViewport:
                    onCentreViewportControl(self);
                    break;
            }
        }

        // 0x004B253A
        static void onStopStartDropdown(Window& self, const int16_t itemIndex)
        {
            auto item = itemIndex == -1 ? Dropdown::getHighlightedItem() : itemIndex;
            if (item == -1 || item > 2)
            {
                return;
            }

            static constexpr std::pair<StringId, VehicleChangeRunningModeArgs::Mode> itemToGameCommandInfo[3] = {
                { StringIds::cant_stop_string_id, VehicleChangeRunningModeArgs::Mode::stopVehicle },
                { StringIds::cant_start_string_id, VehicleChangeRunningModeArgs::Mode::startVehicle },
                { StringIds::cant_select_manual_mode_string_id, VehicleChangeRunningModeArgs::Mode::driveManually },
            };

            auto [errorTitle, mode] = itemToGameCommandInfo[item];
            GameCommands::setErrorTitle(errorTitle);
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }

            auto args = FormatArguments::common();
            args.skip(6);
            args.push(head->name);
            args.push(head->ordinalNumber);

            VehicleChangeRunningModeArgs vargs{};
            vargs.head = EntityId(self.number);
            vargs.mode = mode;
            GameCommands::doCommand(vargs, GameCommands::Flags::apply);
        }

        static void onCentreViewportDropdown(Window& self, const int16_t itemIndex)
        {
            if (itemIndex <= 0)
            {
                // Centre main window on vehicle, without locking.
                self.viewportCentreMain();
            }

            // Focus main viewport on vehicle
            if (itemIndex == 1)
            {
                auto vehHead = Common::getVehicle(self);
                if (vehHead == nullptr)
                {
                    return;
                }
                Vehicles::Vehicle train(*vehHead);
                EntityId targetEntity = train.veh2->id;

                // Focus viewport on vehicle, with locking.
                auto main = WindowManager::getMainWindow();
                Windows::Main::viewportFocusOnEntity(*main, targetEntity);
            }
        }

        // 0x004B253A
        static void onDropdown(Window& self, [[maybe_unused]] const WidgetIndex_t widgetIndex, const WidgetId id, const int16_t itemIndex)
        {
            switch (id)
            {
                case Widx::kStopStart:
                    onStopStartDropdown(self, itemIndex);
                    break;
                case Widx::kCentreViewport:
                    onCentreViewportDropdown(self, itemIndex);
                    break;
            }
        }

        // 0x004B2545
        static void onToolUpdate(Window& self, const WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id, const int16_t x, const int16_t y)
        {
            if (widgetIndex != widx::pickup)
            {
                return;
            }
            Common::pickupToolUpdate(self, x, y);
        }

        // 0x004B2550
        static void onToolDown(Window& self, const WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id, const int16_t x, const int16_t y)
        {
            if (widgetIndex != widx::pickup)
            {
                return;
            }
            Common::pickupToolDown(self, x, y);
        }

        // 0x004B255B
        static void onToolAbort(Window& self, const WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id)
        {
            if (widgetIndex != widx::pickup)
            {
                return;
            }
            Common::pickupToolAbort(self);
        }

        // 0x004B31F2
        static std::optional<FormatArguments> tooltip(Ui::Window& self, WidgetIndex_t, [[maybe_unused]] const WidgetId id)
        {
            FormatArguments args{};

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return {};
            }
            args.skip(2);
            args.push(StringIds::getVehicleType(head->vehicleType));
            return args;
        }

        // 0x004B1EB5
        static void prepareDraw(Window& self)
        {
            Common::setActiveTabs(self);
            Common::setCaptionEnableState(self);

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            Vehicles::Vehicle train(*head);

            self.widgets[widx::stopStart].hidden = false;
            self.widgets[widx::pickup].hidden = false;
            self.widgets[widx::passSignal].hidden = false;
            self.widgets[widx::changeDirection].hidden = false;

            if (head->mode != TransportMode::rail)
            {
                self.widgets[widx::passSignal].hidden = true;
            }

            if (head->mode == TransportMode::air || head->mode == TransportMode::water)
            {
                self.widgets[widx::changeDirection].hidden = true;
            }

            self.disabledWidgets &= ~interactiveWidgets;

            auto veh1 = train.veh1;
            if (train.cars.empty())
            {
                self.disabledWidgets |= (1 << widx::pickup);
            }

            if (veh1->var_3C >= 0x3689)
            {
                self.disabledWidgets |= (1 << widx::pickup) | (1 << widx::changeDirection);
            }

            if (head->mode == TransportMode::air || head->mode == TransportMode::water)
            {
                if (head->status != Vehicles::Status::stopped && head->status != Vehicles::Status::loading && head->tileX != -1)
                {
                    self.disabledWidgets |= (1 << widx::pickup);
                }
            }

            if (head->tileX == -1)
            {
                self.disabledWidgets |= (1 << widx::stopStart) | (1 << widx::passSignal) | (1 << widx::changeDirection) | (1 << widx::centreViewport);
                if (ToolManager::isToolActive(WindowType::vehicle, self.number))
                {
                    self.disabledWidgets &= ~(1 << widx::changeDirection); //???
                }
            }

            if (head->status != Vehicles::Status::waitingAtSignal)
            {
                self.disabledWidgets |= (1 << widx::passSignal);
            }

            auto company = CompanyManager::get(head->owner);

            // Set title.
            auto args = FormatArguments(self.widgets[Common::widx::caption].textArgs);
            if (CompanyManager::isPlayerCompany(head->owner))
            {
                args.push(StringIds::company_vehicle);
            }
            else
            {
                args.push(StringIds::competitor_vehicle);
            }
            args.push(company->name);
            args.skip(2);
            args.push(head->name);
            args.push(head->ordinalNumber);

            uint32_t stopStartImage = ImageIds::red_flag;
            if (head->hasVehicleFlags(Vehicles::VehicleFlags::manualControl))
            {
                stopStartImage = ImageIds::yellow_flag;
            }
            else if (head->hasVehicleFlags(Vehicles::VehicleFlags::commandStop))
            {
                stopStartImage = ImageIds::red_flag;
            }
            else
            {
                stopStartImage = ImageIds::green_flag;
            }
            self.widgets[widx::stopStart].image = stopStartImage;

            auto [pickupImage, pickupTooltip] = Common::getPickupImageIdandTooltip(*head, head->isPlaced());
            self.widgets[widx::pickup].image = Gfx::recolour(pickupImage);
            self.widgets[widx::pickup].tooltip = pickupTooltip;

            self.widgets[widx::speedControl].hidden = true;

            self.widgets[Common::widx::frame].right = self.width - 1;
            self.widgets[Common::widx::frame].bottom = self.height - 1;
            self.widgets[Common::widx::panel].right = self.width - 1;
            self.widgets[Common::widx::panel].bottom = self.height - 1;
            self.widgets[Common::widx::caption].right = self.width - 2;
            self.widgets[Common::widx::closeButton].left = self.width - 15;
            self.widgets[Common::widx::closeButton].right = self.width - 3;

            int viewportRight = self.width - 26;
            if (head->hasVehicleFlags(Vehicles::VehicleFlags::manualControl))
            {
                if (CompanyManager::isPlayerCompany(head->owner))
                {
                    viewportRight -= 27;
                    self.widgets[widx::speedControl].hidden = false;
                }
            }

            self.widgets[widx::viewport].right = viewportRight;
            self.widgets[widx::viewport].bottom = self.height - 1 - 13;

            self.widgets[widx::status].top = self.height - 1 - 13 + 2;
            self.widgets[widx::status].bottom = self.height - 1 - 13 + 2 + 9;
            self.widgets[widx::status].right = self.width - 14;

            self.widgets[widx::stopStart].right = self.width - 2;
            self.widgets[widx::pickup].right = self.width - 2;
            self.widgets[widx::passSignal].right = self.width - 2;
            self.widgets[widx::changeDirection].right = self.width - 2;

            self.widgets[widx::stopStart].left = self.width - 2 - 23;
            self.widgets[widx::pickup].left = self.width - 2 - 23;
            self.widgets[widx::passSignal].left = self.width - 2 - 23;
            self.widgets[widx::changeDirection].left = self.width - 2 - 23;

            self.widgets[widx::speedControl].left = self.width - 2 - 23 - 26;
            self.widgets[widx::speedControl].right = self.width - 2 - 23 - 26 + 23;

            if (!CompanyManager::isPlayerCompany(head->owner))
            {
                self.widgets[widx::stopStart].hidden = true;
                self.widgets[widx::pickup].hidden = true;
                self.widgets[widx::passSignal].hidden = true;
                self.widgets[widx::changeDirection].hidden = true;
                self.widgets[widx::viewport].right += 22;
            }

            self.widgets[widx::centreViewport].right = self.widgets[widx::viewport].right - 1;
            self.widgets[widx::centreViewport].bottom = self.widgets[widx::viewport].bottom - 1;
            self.widgets[widx::centreViewport].left = self.widgets[widx::viewport].right - 1 - 23;
            self.widgets[widx::centreViewport].top = self.widgets[widx::viewport].bottom - 1 - 23;
            Widget::leftAlignTabs(self, Common::widx::tabMain, Common::widx::tabRoute);
        }

        // 0x004B226D
        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);

            Widget& pickupButton = self.widgets[widx::pickup];
            if (!pickupButton.hidden)
            {
                if ((pickupButton.image & 0x20000000) != 0 && !self.isDisabled(widx::pickup))
                {
                    drawingCtx.drawImage(ZoomLevel::full, pickupButton.left, pickupButton.top, Gfx::recolour(pickupButton.image, CompanyManager::getCompanyColour(self.owner)));
                }
            }

            auto veh = Common::getVehicle(self);
            if (veh == nullptr)
            {
                return;
            }
            {
                auto status = veh->getStatus();
                FormatArguments args = {};
                args.setTransportMode(enumValue(veh->mode));
                args.push(status.status1);
                args.push(status.status1Args);
                args.push(status.status2);
                args.push(status.status2Args);

                StringId strFormat = StringIds::black_stringid_stringid;
                if (status.status2 == StringIds::null)
                {
                    strFormat = StringIds::black_stringid;
                }

                auto& widget = self.widgets[widx::status];
                auto point = Point(widget.left - 1, widget.top - 1);
                tr.drawStringLeftClipped(point, widget.width() - 1, Colour::black, strFormat, args);
            }

            Widget& speedWidget = self.widgets[widx::speedControl];
            if (!speedWidget.hidden)
            {
                drawingCtx.drawImage(ZoomLevel::full, speedWidget.left, speedWidget.top + 10, Gfx::recolour(ImageIds::speed_control_track, self.getColour(WindowColour::secondary).c()));

                auto point = Point(speedWidget.midX(), speedWidget.top + 4);
                tr.drawStringCentred(point, Colour::black, StringIds::tiny_power);

                point = Point(speedWidget.midX(), speedWidget.bottom - 10);
                tr.drawStringCentred(point, Colour::black, StringIds::tiny_brake);

                drawingCtx.drawImage(ZoomLevel::full, speedWidget.left + 1, speedWidget.top + 57 - veh->manualPower, Gfx::recolour(ImageIds::speed_control_thumb, self.getColour(WindowColour::secondary).c()));
            }

            if (self.viewports[0] == nullptr && ToolManager::isToolActive(self.type, self.number))
            {
                FormatArguments args = {};
                args.push(StringIds::getVehicleType(veh->vehicleType));

                auto& button = self.widgets[widx::viewport];
                auto origin = Point(button.midX(), button.midY());
                tr.drawStringCentredWrapped(origin, button.width() - 6, Colour::black, StringIds::click_on_view_select_string_id_start, args);
            }
        }

        static constexpr WindowEventList kEvents = {
            .onClose = Common::onClose,
            .onMouseUp = onMouseUp,
            .onResize = onResize,
            .onMouseDown = onMouseDown,
            .onDropdown = onDropdown,
            .onUpdate = onUpdate,
            .onToolUpdate = onToolUpdate,
            .onToolDown = onToolDown,
            .onToolAbort = onToolAbort,
            .textInput = Common::textInput,
            .viewportRotate = createViewport,
            .tooltip = tooltip,
            .prepareDraw = prepareDraw,
            .draw = draw,
        };

        static const WindowEventList& getEvents()
        {
            return kEvents;
        }
    }

    namespace Details
    {
        // 0x4B60CC
        Window* open(const Vehicles::VehicleBase* vehicle)
        {
            auto self = Main::open(vehicle);
            if (self != nullptr)
            {
                self->callOnMouseUp(Common::widx::tabDetails, self->widgets[Common::widx::tabDetails].id);
            }
            return self;
        }

        static void cloneVehicle(Window& self, const bool shareOrders)
        {
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }

            GameCommands::setErrorTitle(StringIds::cant_clone_vehicle);

            GameCommands::VehicleCloneArgs args{};
            args.vehicleHeadId = head->head;
            args.shareOrders = shareOrders;

            if (GameCommands::doCommand(args, GameCommands::Flags::apply) != GameCommands::kFailure)
            {
                auto* newVehicle = EntityManager::get<Vehicles::VehicleBase>(GameCommands::getLegacyReturnState().lastCreatedVehicleId);
                if (newVehicle != nullptr)
                {
                    Windows::Vehicle::Details::open(newVehicle);
                }
            }
        }

        // 0x004B3823
        static void onMouseUp(Window& self, WidgetIndex_t widgetIndex, const WidgetId id)
        {
            switch (id)
            {
                case Common::Widx::kCloseButton:
                    WindowManager::close(&self);
                    break;
                case Common::Widx::kCaption:
                    Common::renameVehicle(self, widgetIndex);
                    break;
                case Common::Widx::kTabMain:
                case Common::Widx::kTabDetails:
                case Common::Widx::kTabCargo:
                case Common::Widx::kTabFinances:
                case Common::Widx::kTabRoute:
                    Common::switchTab(self, widgetIndex);
                    break;
                case Widx::kPickup:
                    Common::onPickup(self, widx::pickup);
                    break;
                case Widx::kRemove:
                {
                    auto head = Common::getVehicle(self);
                    if (head == nullptr)
                    {
                        break;
                    }
                    FormatArguments args{};
                    args.skip(10);
                    args.push(head->name);
                    args.push(head->ordinalNumber);
                    GameCommands::setErrorTitle(StringIds::cant_sell_string_id);

                    GameCommands::VehicleSellArgs gcArgs{};
                    gcArgs.car = head->id;

                    GameCommands::doCommand(gcArgs, GameCommands::Flags::apply);
                    break;
                }
            }
        }

        // 0x004B3D73
        static void onResize(Window& self)
        {
            if (CompanyManager::getControllingId() == self.owner)
            {
                self.setSizeBounds(kMinWindowSizeWithPaintEnabled, kMaxWindowSize);
            }
            else
            {
                self.setSizeBounds(kMinWindowSize, kMaxWindowSize);
            }
        }

        static void onMouseDown(Window& self, const WidgetIndex_t widgetIndex, const WidgetId id)
        {
            if (id == Widx::kBuildNew)
            {
                Dropdown::add(0, StringIds::dropdown_stringid, StringIds::dropdown_modify_vehicle);
                Dropdown::add(1, StringIds::dropdown_stringid, StringIds::dropdown_clone_vehicle_shared_orders_hint);

                auto& widget = self.widgets[widx::buildNew];
                Dropdown::showText(
                    self.x + widget.left,
                    self.y + widget.top,
                    widget.width(),
                    widget.height(),
                    self.getColour(WindowColour::secondary),
                    2,
                    0);

                Dropdown::setItemSelected(0);
                Dropdown::setHighlightedItem(0);
                return;
            }
            if (id == Widx::kPaintColourPrimary || id == Widx::kPaintColourSecondary)
            {
                auto availableColours = 0x7FFFFFFF;
                Colour selectedColour = ImageId::fromUInt32(self.widgets[widgetIndex].image).getPrimary();
                Dropdown::showColour(&self, &self.widgets[widgetIndex], availableColours, selectedColour, self.getColour(WindowColour::secondary));
                return;
            }
            if (id == Widx::kPaintBrush)
            {
                Dropdown::add(0, StringIds::dropdown_stringid, StringIds::vehicleRepaintTool);
                Dropdown::add(1, StringIds::dropdown_stringid, StringIds::vehicleRepaintEntireVehicle);

                auto& widget = self.widgets[widx::paintBrush];
                Dropdown::showText(
                    self.x + widget.left,
                    self.y + widget.top,
                    widget.width(),
                    widget.height(),
                    self.getColour(WindowColour::secondary),
                    2,
                    0);

                Dropdown::setItemSelected(0);
                Dropdown::setHighlightedItem(0);
                return;
            }
        }

        // 0x004B253A
        static void onDropdown(Window& self, const WidgetIndex_t widgetIndex, const WidgetId id, const int16_t itemIndex)
        {
            if (id == Widx::kBuildNew)
            {
                if (itemIndex <= 0)
                {
                    BuildVehicle::openByVehicleId(EntityId(self.number));
                }
                else if (itemIndex == 1)
                {
                    cloneVehicle(self, Input::hasKeyModifier(Input::KeyModifier::control));
                }
                return;
            }
            if (id == Widx::kPaintColourPrimary || id == Widx::kPaintColourSecondary)
            {
                if (itemIndex == -1)
                {
                    return;
                }
                Colour selectedColour = static_cast<Colour>(Dropdown::getItemArgument(itemIndex, 2));
                self.widgets[widgetIndex].image = Widget::kImageIdColourSet | ImageId::fromUInt32(ImageIds::colour_swatch_recolourable).withPrimary(selectedColour).toUInt32();
                self.invalidate();
                return;
            }
            if (id == Widx::kPaintBrush)
            {
                if (itemIndex == 1 || Input::hasKeyModifier(Input::KeyModifier::shift))
                {
                    paintEntireTrain(self);
                    if (!isPaintToolActive(self))
                    {
                        paintToolBegin(self);
                    }
                }
                else if (itemIndex <= 0)
                {
                    paintToolBegin(self);
                }
                return;
            }
        }

        // 0x004B3C45
        // "Show <vehicle> design details and options" tab in vehicle window
        static void onUpdate(Window& self)
        {
            self.frameNo += 1;
            self.callPrepareDraw();

            WindowManager::invalidateWidget(WindowType::vehicle, self.number, Common::widx::tabDetails);

            if (WindowManager::find(WindowType::dragVehiclePart) == nullptr && self.isActivated(widx::remove))
            {
                self.activatedWidgets &= ~(1ULL << widx::remove);
                WindowManager::invalidateWidget(WindowType::vehicle, self.number, widx::remove);
            }

            if (self.isDisabled(widx::pickup))
            {
                ToolManager::toolCancel(WindowType::vehicle, self.number);
                return;
            }

            auto vehicle = Common::getVehicle(self);
            if (vehicle == nullptr)
            {
                return;
            }
            if (vehicle->isPlaced())
            {
                return;
            }

            if (!WindowManager::isInFrontAlt(&self))
            {
                return;
            }

            if (vehicle->owner != CompanyManager::getControllingId())
            {
                return;
            }

            if (!ToolManager::isToolActive(WindowType::vehicle, self.number))
            {
                Common::onPickup(self, widx::pickup);
            }
        }

        // 0x004B385F
        static void onToolUpdate(Window& self, const WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id, const int16_t x, const int16_t y)
        {
            switch (widgetIndex)
            {
                case widx::pickup:
                    Common::pickupToolUpdate(self, x, y);
                    break;
                default:
                    break;
            }
        }

        // 0x004B386A
        static void onToolDown(Window& self, const WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id, const int16_t x, const int16_t y)
        {
            switch (widgetIndex)
            {
                case widx::pickup:
                    Common::pickupToolDown(self, x, y);
                    break;
                case widx::paintBrush:
                    paintToolDown(self, x, y);
                    break;
                default:
                    break;
            }
        }

        // 0x004B3875
        static void onToolAbort(Window& self, const WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id)
        {
            switch (widgetIndex)
            {
                case widx::pickup:
                    Common::pickupToolAbort(self);
                    break;
                case widx::paintBrush:
                    paintToolAbort(self);
                    break;
                default:
                    break;
            }
        }

        // 0x4B38FA
        static void getScrollSize(Ui::Window& self, [[maybe_unused]] const uint32_t scrollIndex, [[maybe_unused]] int32_t& scrollWidth, int32_t& scrollHeight)
        {
            scrollHeight = static_cast<int32_t>(Common::getNumCars(self) * self.rowHeight);
        }

        // 0x004B3B54
        static void scrollMouseDown(Window& self, [[maybe_unused]] const int16_t x, const int16_t y, [[maybe_unused]] const uint8_t scrollIndex)
        {
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            if (head->owner != CompanyManager::getControllingId())
            {
                return;
            }

            auto car = Common::getCarFromScrollView(self, y);
            if (!car)
            {
                return;
            }

            OpenLoco::Vehicles::Vehicle train{ *head };
            for (auto c : train.cars)
            {
                if (c.front == car->front)
                {
                    if (isPaintToolActive(self))
                    {
                        paintToolDownScroll(self, c, x);
                        return;
                    }
                    DragVehiclePart::open(c);
                    break;
                }
            }
        }

        // 0x004B399E
        static void scrollMouseOver(Window& self, [[maybe_unused]] const int16_t x, const int16_t y, [[maybe_unused]] const uint8_t scrollIndex)
        {
            Ui::ToolTip::setTooltipTimeout(2000);
            self.flags &= ~WindowFlags::notScrollView;
            auto car = Common::getCarFromScrollView(self, y);
            StringId tooltipFormat = StringIds::null;
            EntityId tooltipContent = EntityId::null;
            if (car)
            {
                tooltipFormat = StringIds::buffer_337;
                tooltipContent = car->front->id;
            }
            if (EntityId(self.rowHover) != tooltipContent)
            {
                self.rowHover = enumValue(tooltipContent);
                self.invalidate();
            }

            char* buffer = const_cast<char*>(StringManager::getString(StringIds::buffer_337));
            if (StringManager::locoStrlen(buffer) != 0)
            {
                if (self.widgets[widx::carList].tooltip == tooltipFormat && self.var_85C == enumValue(tooltipContent))
                {
                    return;
                }
            }

            self.widgets[widx::carList].tooltip = tooltipFormat;
            self.var_85C = enumValue(tooltipContent);
            ToolTip::closeAndReset();

            if (tooltipContent == EntityId::null)
            {
                return;
            }

            Ui::ToolTip::set_52336E(true);

            auto vehicleObj = ObjectManager::get<VehicleObject>(car->front->objectId);
            {
                FormatArguments args{};
                args.push(vehicleObj->name);
                buffer = StringManager::formatString(buffer, StringIds::tooltip_stringid, args);
            }

            {
                FormatArguments args{};
                args.push(car->front->creationDay);
                buffer = StringManager::formatString(buffer, StringIds::vehicle_details_tooltip_built, args);
            }

            if (vehicleObj->power != 0 && (vehicleObj->mode == TransportMode::road || vehicleObj->mode == TransportMode::rail))
            {
                FormatArguments args{};
                args.push(vehicleObj->power);
                buffer = StringManager::formatString(buffer, StringIds::vehicle_details_tooltip_power, args);
            }

            if (vehicleObj->mode == TransportMode::rail || vehicleObj->mode == TransportMode::road)
            {
                FormatArguments args{};
                args.push<uint32_t>(StringManager::internalLengthToComma1DP(vehicleObj->getLength()));
                buffer = StringManager::formatString(buffer, StringIds::vehicle_details_tooltip_length, args);
            }

            {
                FormatArguments args{};
                args.push(vehicleObj->weight);
                buffer = StringManager::formatString(buffer, StringIds::vehicle_details_tooltip_weight, args);
            }

            {
                FormatArguments args{};
                args.setTransportMode(enumValue(vehicleObj->mode));
                args.push(vehicleObj->speed);
                buffer = StringManager::formatString(buffer, StringIds::vehicle_details_tooltip_max_speed, args);
            }

            if (vehicleObj->hasFlags(VehicleObjectFlags::rackRail))
            {
                FormatArguments args{};
                args.setTransportMode(enumValue(vehicleObj->mode));
                args.push(vehicleObj->rackSpeed);
                auto rackRailObj = ObjectManager::get<TrackExtraObject>(vehicleObj->rackRailType);
                args.push(rackRailObj->name);
                buffer = StringManager::formatString(buffer, StringIds::vehicle_details_tooltip_speed_on_stringid, args);
            }

            {
                FormatArguments args{};
                args.push(car->front->refundCost);
                buffer = StringManager::formatString(buffer, StringIds::vehicle_details_tooltip_value, args);
            }

            if (car->front->reliability != 0)
            {
                FormatArguments args{};
                args.push(car->front->reliability / 256);
                buffer = StringManager::formatString(buffer, StringIds::vehicle_details_tooltip_reliability, args);
            }
        }

        // 0x004B3880 TODO: common across 3 tabs
        static std::optional<FormatArguments> tooltip(Ui::Window& self, WidgetIndex_t, [[maybe_unused]] const WidgetId id)
        {
            FormatArguments args{};
            args.push(StringIds::tooltip_scroll_vehicle_list);

            auto vehicle = Common::getVehicle(self);
            if (vehicle == nullptr)
            {
                return {};
            }
            args.push(StringIds::getVehicleType(vehicle->vehicleType));
            return args;
        }

        // 0x004B3B18
        static Ui::CursorId cursor(Window& self, [[maybe_unused]] const WidgetIndex_t widgetIdx, const WidgetId id, [[maybe_unused]] const int16_t x, const int16_t y, const Ui::CursorId fallback)
        {
            if (id != Widx::kCarList)
            {
                return fallback;
            }

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return fallback;
            }
            if (head->owner != CompanyManager::getControllingId())
            {
                return fallback;
            }

            auto selectedCar = Common::getCarFromScrollView(self, y);
            if (!selectedCar)
            {
                return fallback;
            }
            if (isPaintToolActive(self))
            {
                return CursorId::brush;
            }
            return CursorId::openHand;
        }

        static constexpr auto kAdditionalVehicleButtonByVehicleType = Utility::buildLookupTable<VehicleType, uint32_t>({
            { VehicleType::train, InterfaceSkin::ImageIds::build_additional_train },
            { VehicleType::bus, InterfaceSkin::ImageIds::build_additional_bus },
            { VehicleType::truck, InterfaceSkin::ImageIds::build_additional_truck },
            { VehicleType::tram, InterfaceSkin::ImageIds::build_additional_tram },
            { VehicleType::aircraft, InterfaceSkin::ImageIds::build_additional_aircraft },
            { VehicleType::ship, InterfaceSkin::ImageIds::build_additional_ship },
        });

        constexpr auto kVehicleDetailsOffset = 2;
        constexpr auto kVehicleDetailsLineHeight = 12;
        constexpr auto kVehicleDetailsTextHeight = kVehicleDetailsOffset + kVehicleDetailsLineHeight * 3;

        static void alignToRightBar(Window& self, widx widget)
        {
            self.widgets[widget].left = self.width - 25;
            self.widgets[widget].right = self.width - 2;
        }

        // 0x004B3300
        static void prepareDraw(Window& self)
        {
            Common::setActiveTabs(self);
            Common::setCaptionEnableState(self);

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }

            // Set title.
            {
                auto args = FormatArguments(self.widgets[Common::widx::caption].textArgs);
                args.push(head->name);
                args.push(head->ordinalNumber);
            }

            self.widgets[Common::widx::frame].right = self.width - 1;
            self.widgets[Common::widx::frame].bottom = self.height - 1;
            self.widgets[Common::widx::panel].right = self.width - 1;
            self.widgets[Common::widx::panel].bottom = self.height - 1;
            self.widgets[Common::widx::caption].right = self.width - 2;
            self.widgets[Common::widx::closeButton].left = self.width - 15;
            self.widgets[Common::widx::closeButton].right = self.width - 3;
            Widget::leftAlignTabs(self, Common::widx::tabMain, Common::widx::tabRoute);

            self.widgets[widx::carList].right = self.width - 26;
            self.widgets[widx::carList].bottom = self.height - kVehicleDetailsTextHeight;
            alignToRightBar(self, widx::buildNew);
            alignToRightBar(self, widx::pickup);
            alignToRightBar(self, widx::remove);
            alignToRightBar(self, widx::paintBrush);

            if (isPaintToolActive(self))
            {
                self.activatedWidgets |= (1U << widx::paintBrush);
                self.widgets[widx::carList].bottom = self.height - kVehicleDetailsTextHeight;

                self.widgets[widx::paintColourPrimary].hidden = false;
                self.widgets[widx::paintColourPrimary].right = self.width - 23;
                self.widgets[widx::paintColourPrimary].left = self.width - 39;
                self.widgets[widx::paintColourPrimary].bottom = self.height - 17;
                self.widgets[widx::paintColourPrimary].top = self.height - 33;

                self.widgets[widx::paintColourSecondary].hidden = false;
                self.widgets[widx::paintColourSecondary].right = self.width - 5;
                self.widgets[widx::paintColourSecondary].left = self.width - 21;
                self.widgets[widx::paintColourSecondary].bottom = self.height - 17;
                self.widgets[widx::paintColourSecondary].top = self.height - 33;
            }
            else
            {
                self.activatedWidgets &= ~(1U << widx::paintBrush);

                self.widgets[widx::paintColourPrimary].hidden = true;
                self.widgets[widx::paintColourSecondary].hidden = true;
            }

            // Differs to main tab! Unsure why.
            if (head->isPlaced())
            {
                self.widgets[widx::pickup].hidden = true;
            }
            if (head->owner != CompanyManager::getControllingId())
            {
                self.widgets[widx::carList].bottom = self.height - kVehicleDetailsTextHeight;
                self.widgets[Details::widx::paintColourPrimary].hidden = true;
                self.widgets[Details::widx::paintColourSecondary].hidden = true;

                self.widgets[widx::buildNew].hidden = true;
                self.widgets[widx::pickup].hidden = true;
                self.widgets[widx::remove].hidden = true;

                self.widgets[widx::carList].right = self.width - 4;
                self.widgets[widx::paintBrush].hidden = true;
                self.widgets[widx::paintBrush].content = Widget::kContentNull;
            }
            else
            {
                self.widgets[widx::buildNew].hidden = false;
                self.widgets[widx::pickup].hidden = false;
                self.widgets[widx::remove].hidden = false;

                self.widgets[widx::paintBrush].bottom = self.height - kVehicleDetailsTextHeight;
                self.widgets[widx::paintBrush].top = self.height - kVehicleDetailsTextHeight - 24;
            }

            auto skin = ObjectManager::get<InterfaceSkinObject>();
            auto buildImage = skin->img + kAdditionalVehicleButtonByVehicleType.at(head->vehicleType);

            self.widgets[widx::buildNew].image = Gfx::recolour(buildImage, CompanyManager::getCompanyColour(self.owner)) | Widget::kImageIdColourSet;

            Vehicles::Vehicle train{ *head };
            if (train.cars.empty())
            {
                self.disabledWidgets |= 1 << widx::pickup;
            }
            else
            {
                self.disabledWidgets &= ~(1ULL << widx::pickup);
            }

            auto [pickupImage, pickupTooltip] = Common::getPickupImageIdandTooltip(*head, head->isPlaced());
            self.widgets[widx::pickup].image = Gfx::recolour(pickupImage);
            self.widgets[widx::pickup].tooltip = pickupTooltip;
        }

        static void paintEntireTrain(Window& self)
        {
            GameCommands::VehicleRepaintArgs args{};
            args.paintFlags = GameCommands::VehicleRepaintFlags::paintFromVehicleUi | GameCommands::VehicleRepaintFlags::applyToEntireTrain;
            args.head = EntityId(self.number);
            args.setColours(getPaintToolColour(self));
            GameCommands::doCommand(args, GameCommands::Flags::apply);
            self.invalidate();
        }

        static void paintToolDown(Window& self, const int16_t x, const int16_t y)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            auto interaction = ViewportInteraction::getItemLeft(x, y);
            if (interaction.type == ViewportInteraction::InteractionItem::entity)
            {
                auto _entity = reinterpret_cast<EntityBase*>(interaction.object);
                auto veh = _entity->asBase<Vehicles::VehicleBase>();

                if (veh->getHead() != EntityId(self.number))
                {
                    return;
                }

                GameCommands::VehicleRepaintArgs args{};
                args.paintFlags = GameCommands::VehicleRepaintFlags::paintFromVehicleUi;
                if (Input::hasKeyModifier(Input::KeyModifier::control))
                {
                    args.paintFlags ^= GameCommands::VehicleRepaintFlags::applyToEntireCar;
                }
                if (Input::hasKeyModifier(Input::KeyModifier::shift))
                {
                    args.paintFlags |= GameCommands::VehicleRepaintFlags::applyToEntireTrain;
                }

                args.setColours(getPaintToolColour(self));
                args.head = veh->id;

                GameCommands::doCommand(args, GameCommands::Flags::apply);
                self.invalidate();
            }
        }

        static void paintToolDownScroll(Window& self, Vehicles::Car car, const int16_t x)
        {
            GameCommands::VehicleRepaintArgs args{};
            args.paintFlags = GameCommands::VehicleRepaintFlags::paintFromVehicleUi;
            args.setColours(getPaintToolColour(self));
            args.head = car.front->id;

            if (Input::hasKeyModifier(Input::KeyModifier::shift))
            {
                args.paintFlags |= GameCommands::VehicleRepaintFlags::applyToEntireTrain;
            }

            if (Input::hasKeyModifier(Input::KeyModifier::control) && !args.hasRepaintFlags(GameCommands::VehicleRepaintFlags::applyToEntireTrain))
            {
                auto obj = ObjectManager::get<VehicleObject>(car.front->objectId);

                if (obj == nullptr)
                {
                    return;
                }

                constexpr const uint8_t drawYaw = 40;

                BodyItems items = getBodyItemsForVehicle(*obj, drawYaw, car);
                auto cursorPosX = x - self.widgets[widx::carList].left;
                for (BodyItem& body : items.items)
                {
                    auto g1Elem = Gfx::getG1Element(body.image);
                    auto spriteRight = body.dist + g1Elem->xOffset + g1Elem->width;
                    if (cursorPosX <= spriteRight)
                    {
                        args.head = body.body;
                        args.paintFlags ^= GameCommands::VehicleRepaintFlags::applyToEntireCar;
                        break;
                    }
                }
            }

            GameCommands::doCommand(args, GameCommands::Flags::apply);
            self.invalidate();
        }

        static bool isPaintToolActive(Window& self)
        {
            return ToolManager::isToolActive(self.type, self.number, widx::paintBrush);
        }

        static void paintToolBegin(Window& self)
        {
            bool active = ToolManager::toolSet(self, widx::paintBrush, CursorId::brush);
            self.activatedWidgets &= ~(1U << widx::paintBrush);
            self.activatedWidgets |= (1U << widx::paintBrush) * active;
            self.invalidate();
        }

        static void paintToolAbort(Window& self)
        {
            ToolManager::toolCancel(self.type, self.number);
            self.activatedWidgets &= ~(1U << widx::paintBrush);
            self.invalidate();
        }

        static ColourScheme getPaintToolColour(Window& self)
        {
            return { ImageId(self.widgets[widx::paintColourPrimary].image).getPrimary(), ImageId(self.widgets[widx::paintColourSecondary].image).getPrimary() };
        }

        // 0x004B3542
        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);

            // TODO: identical to main tab (doesn't appear to do anything useful)
            if (!self.widgets[widx::pickup].hidden)
            {
                if ((self.widgets[widx::pickup].image & (1 << 29)) && !self.isDisabled(widx::pickup))
                {
                    auto image = Gfx::recolour(self.widgets[widx::pickup].image, CompanyManager::getCompanyColour(self.owner));
                    drawingCtx.drawImage(ZoomLevel::full, self.widgets[widx::pickup].left, self.widgets[widx::pickup].top, image);
                }
            }
            uint16_t textRightEdge = isPaintToolActive(self) ? self.width - 39 : self.width;

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }

            Ui::Point pos = { 3, self.height - kVehicleDetailsTextHeight + kVehicleDetailsOffset };
            Vehicles::Vehicle train{ *head };

            // Draw power and weight
            {
                FormatArguments args{};
                args.push(train.veh2->totalPower);
                args.push<uint32_t>(train.veh2->totalWeight);

                StringId str = StringIds::vehicle_details_weight;
                if (train.veh2->mode == TransportMode::rail || train.veh2->mode == TransportMode::road)
                {
                    str = StringIds::vehicle_details_total_power_and_weight;
                }
                tr.drawStringLeftClipped(pos, std::min<uint16_t>(self.width - 6, textRightEdge), Colour::black, str, args);
            }

            // Draw max (rack rail) speed and reliability
            {
                pos.y += kVehicleDetailsLineHeight;
                FormatArguments args{};
                args.setTransportMode(enumValue(train.veh2->mode));
                args.push<uint16_t>(train.veh2->maxSpeed == kSpeed16Null ? 0 : train.veh2->maxSpeed.getRaw());
                args.push<uint16_t>(train.veh2->rackRailMaxSpeed == kSpeed16Null ? 0 : train.veh2->rackRailMaxSpeed.getRaw());
                args.push<uint16_t>(train.veh2->reliability == 0 ? 64 : train.veh2->reliability);

                StringId str = StringIds::vehicle_details_max_speed_and_reliability;
                if (train.veh1->var_49 != 0)
                {
                    str = StringIds::vehicle_details_max_speed_and_rack_rail_and_reliability;
                }
                tr.drawStringLeftClipped(pos, std::min<uint16_t>(self.width - 16, textRightEdge), Colour::black, str, args);
            }

            // Draw car count and vehicle length
            {
                pos.y += kVehicleDetailsLineHeight;
                FormatArguments args = {};
                StringId str = StringIds::vehicle_length;
                args.push<uint32_t>(StringManager::internalLengthToComma1DP(head->getVehicleTotalLength()));
                if (train.veh2->mode == TransportMode::rail && head->getCarCount() > 1)
                {
                    str = StringIds::vehicle_car_count_and_length;
                    args.push<uint32_t>(head->getCarCount());
                }
                tr.drawStringLeftClipped(pos, std::min<uint16_t>(self.width - 16, textRightEdge), Colour::black, str, args);
            }
        }

        // 0x004B36A3
        static void drawScroll(Window& self, Gfx::DrawingContext& drawingCtx, [[maybe_unused]] const uint32_t i)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            drawingCtx.clearSingle(Colours::getShade(self.getColour(WindowColour::secondary).c(), 4));
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            OpenLoco::Vehicles::Vehicle train{ *head };
            Ui::Point pos{ 0, 0 };
            for (auto& car : train.cars)
            {
                StringId carStr = StringIds::black_stringid;
                if (EntityId(self.rowHover) == car.front->id)
                {
                    carStr = StringIds::wcolour2_stringid;

                    int16_t top = pos.y;
                    int16_t bottom = pos.y + self.rowHeight - 1;
                    if (DragVehiclePart::getDragCarComponent() != nullptr)
                    {
                        bottom = pos.y;
                        top = pos.y - 1;
                        carStr = StringIds::black_stringid;
                    }
                    drawingCtx.fillRect(0, top, self.width, bottom, enumValue(ExtColour::unk30), Gfx::RectFlags::transparent);
                }

                int16_t y = pos.y + (self.rowHeight - 22) / 2;
                const auto disableColour = car.front == DragVehiclePart::getDragCarComponent()
                    ? std::make_optional(self.getColour(WindowColour::secondary).c())
                    : std::nullopt;
                drawVehicleInline(drawingCtx, car, { 0, y }, VehicleInlineMode::basic, VehiclePartsToDraw::bogies, disableColour);
                auto x = drawVehicleInline(drawingCtx, car, { 0, y }, VehicleInlineMode::basic, VehiclePartsToDraw::bodies, disableColour);

                auto vehicleObj = ObjectManager::get<VehicleObject>(car.front->objectId);
                FormatArguments args{};
                args.push(vehicleObj->name);
                x += 2;
                y = pos.y + (self.rowHeight / 2) - 6;

                {
                    auto point = Point(x, y);
                    tr.drawStringLeft(point, Colour::black, carStr, args);
                }

                pos.y += self.rowHeight;
            }

            if (EntityId(self.rowHover) == train.tail->id && DragVehiclePart::getDragCarComponent() != nullptr)
            {
                drawingCtx.fillRect(0, pos.y - 1, self.width, pos.y, enumValue(ExtColour::unk30), Gfx::RectFlags::transparent);
            }
        }

        static constexpr WindowEventList kEvents = {
            .onClose = Common::onClose,
            .onMouseUp = onMouseUp,
            .onResize = onResize,
            .onMouseDown = onMouseDown,
            .onDropdown = onDropdown,
            .onUpdate = onUpdate,
            .onHandleInputBegin = listWindowOnHandleInputBegin,
            .onHandleInputEnd = listWindowOnHandleInputEnd,
            .onToolUpdate = onToolUpdate,
            .onToolDown = onToolDown,
            .onToolAbort = onToolAbort,
            .getScrollSize = getScrollSize,
            .scrollMouseDown = scrollMouseDown,
            .scrollMouseOver = scrollMouseOver,
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

        static Ui::Window* getVehicleDetailsWindow(const Ui::Point& pos)
        {
            auto vehicleWindow = WindowManager::findAt(pos);
            if (vehicleWindow == nullptr || vehicleWindow->type != WindowType::vehicle || vehicleWindow->currentTab != (Common::widx::tabDetails - Common::widx::tabMain))
            {
                return nullptr;
            }
            return vehicleWindow;
        }

        static Vehicles::VehicleBase* getCarFromScrollViewPos(Ui::Window& self, const Ui::Point& pos)
        {
            auto res = Ui::ScrollView::getPart(self, &self.widgets[widx::carList], pos.x, pos.y);
            if (res.area != ScrollPart::view)
            {
                return nullptr;
            }

            auto y = self.rowHeight / 2 + res.scrollviewLoc.y;
            auto car = Common::getCarFromScrollView(self, y);
            if (!car)
            {
                auto head = Common::getVehicle(self);
                if (head == nullptr)
                {
                    return nullptr;
                }
                Vehicles::Vehicle train(*head);
                return train.tail;
            }
            return car->front;
        }

        void scrollDrag(const Ui::Point& pos)
        {
            auto vehicleWindow = getVehicleDetailsWindow(pos);
            if (vehicleWindow == nullptr)
            {
                return;
            }
            auto targetWidget = vehicleWindow->findWidgetAt(pos.x, pos.y);
            switch (targetWidget)
            {
                case widx::remove:
                {
                    if (!vehicleWindow->isActivated(widx::remove))
                    {
                        vehicleWindow->activatedWidgets |= (1 << widx::remove);
                        WindowManager::invalidateWidget(WindowType::vehicle, vehicleWindow->number, widx::remove);
                    }
                    return;
                }
                case widx::carList:
                {
                    auto car = getCarFromScrollViewPos(*vehicleWindow, pos);
                    if (car != nullptr)
                    {
                        vehicleWindow->flags &= ~WindowFlags::notScrollView;
                        if (car->id != EntityId(vehicleWindow->rowHover))
                        {
                            vehicleWindow->rowHover = enumValue(car->id);
                        }
                        vehicleWindow->invalidate();
                    }

                    // TODO: define constant for hot zone
                    if (pos.y < vehicleWindow->widgets[widx::carList].top + vehicleWindow->y + 5)
                    {
                        Ui::ScrollView::verticalNudgeUp(*vehicleWindow, vehicleWindow->getScrollDataIndex(widx::carList), widx::carList);
                    }
                    else if (pos.y > vehicleWindow->widgets[widx::carList].bottom + vehicleWindow->y - 5)
                    {
                        Ui::ScrollView::verticalNudgeDown(*vehicleWindow, vehicleWindow->getScrollDataIndex(widx::carList), widx::carList);
                    }
                    break;
                }
            }
            if (vehicleWindow->isActivated(widx::remove))
            {
                vehicleWindow->activatedWidgets &= ~(1ULL << widx::remove);
                WindowManager::invalidateWidget(WindowType::vehicle, vehicleWindow->number, widx::remove);
            }
        }

        void scrollDragEnd(const Ui::Point& pos)
        {
            if (DragVehiclePart::getDragCarComponent() == nullptr)
            {
                return;
            }

            auto vehicleWindow = getVehicleDetailsWindow(pos);
            if (vehicleWindow == nullptr)
            {
                return;
            }

            auto targetWidget = vehicleWindow->findWidgetAt(pos.x, pos.y);
            switch (targetWidget)
            {
                case widx::remove:
                {
                    GameCommands::VehicleSellArgs gcArgs{};
                    gcArgs.car = DragVehiclePart::getDragCarComponent()->id;

                    if (Common::confirmComponentChange(gcArgs.car, StringIds::confirm_vehicle_component_sell_cargo_warning_title, StringIds::confirm_vehicle_component_sell_cargo_warning_message, StringIds::confirm_vehicle_component_sell_cargo_warning_confirm))
                    {
                        GameCommands::setErrorTitle(StringIds::cant_sell_vehicle);
                        GameCommands::doCommand(gcArgs, GameCommands::Flags::apply);
                    }

                    break;
                }
                case widx::carList:
                {
                    auto car = getCarFromScrollViewPos(*vehicleWindow, pos);
                    if (car != nullptr)
                    {
                        GameCommands::VehicleRearrangeArgs args{};
                        args.source = DragVehiclePart::getDragCarComponent()->id;
                        args.dest = car->id;

                        auto* srcVehicle = EntityManager::get<Vehicles::VehicleBase>(args.source);
                        auto* destVehicle = EntityManager::get<Vehicles::VehicleBase>(args.dest);

                        bool sameHead = srcVehicle == nullptr || destVehicle == nullptr || srcVehicle->head == destVehicle->head;

                        const StringId warningMessage = sameHead ? StringIds::confirm_vehicle_component_move_cargo_warning_message : StringIds::confirm_vehicle_component_move_cargo_multiple_vehicles_warning_message;

                        if (Common::confirmComponentChange(args.source, args.dest, StringIds::confirm_vehicle_component_move_cargo_warning_title, warningMessage, StringIds::confirm_vehicle_component_move_cargo_warning_confirm))
                        {
                            GameCommands::setErrorTitle(StringIds::cant_move_vehicle);
                            GameCommands::doCommand(args, GameCommands::Flags::apply);
                        }
                    }
                    break;
                }
            }
        }

        // copied and modified from VehicleDraw::getDrawItemsForVehicle
        static BodyItems getBodyItemsForVehicle(const VehicleObject& vehObject, const uint8_t yaw, const Vehicles::Car& car)
        {
            const auto getScreenDistance = [](int32_t gameDist, uint8_t yaw) {
                const auto unk1 = Math::Trigonometry::computeXYVector(gameDist, yaw);

                const auto p1 = World::gameToScreen(World::Pos3(unk1.x, unk1.y, 0), 0);
                // This /4 is meant to be after the compute but we need to carry it to here
                // to keep precision high
                return -p1.x / 4;
            };

            Vehicles::Vehicle train(car.front->head);
            BodyItems bodyItems{};
            const auto isCarReversed = car.body->has38Flags(Vehicles::Flags38::isReversed);
            const auto isAnimated = false;
            uint8_t componentIndex = isCarReversed ? vehObject.numCarComponents - 1 : 0;
            for (auto& carComponent : car)
            {
                auto& componentObject = vehObject.carComponents[componentIndex];
                // 0x01136172
                auto unkDist = isCarReversed ? componentObject.backBogiePosition : componentObject.frontBogiePosition;

                auto carComponentLength = 0;
                if (componentObject.bodySpriteInd != SpriteIndex::null)
                {
                    auto& bodySprites = vehObject.bodySprites[componentObject.bodySpriteInd & ~(1U << 7)];
                    carComponentLength = bodySprites.halfLength * 2;
                }
                auto unk1136174 = isCarReversed ? componentObject.frontBogiePosition : componentObject.backBogiePosition;

                auto bodyDist = bodyItems.totalDistance + (unkDist + carComponentLength - unk1136174) / 2;
                if (carComponent.body->objectSpriteType != SpriteIndex::null)
                {
                    auto& bodySprites = vehObject.bodySprites[carComponent.body->objectSpriteType];

                    auto unk = yaw;
                    if (carComponent.body->has38Flags(Vehicles::Flags38::isReversed))
                    {
                        unk ^= 1U << 5;
                    }

                    auto rollIndex = isAnimated ? carComponent.body->animationFrame : 0;
                    rollIndex += carComponent.body->cargoFrame;

                    auto spriteIndex = getBodyImageIndex(bodySprites, Pitch::flat, unk, rollIndex, 0);
                    bodyItems.items.push_back(BodyItem{ spriteIndex, getScreenDistance(bodyDist, yaw), carComponent.body->id });
                }
                bodyItems.totalDistance += carComponentLength;
                if (isCarReversed)
                {
                    componentIndex--;
                }
                else
                {
                    componentIndex++;
                }
            }
            return bodyItems;
        }
    }

    namespace Cargo
    {
        struct CargoWindowState
        {
            CargoRouteTree::GroupOrder groupOrder = CargoRouteTree::GroupOrder::destinationSourceVia;
            CargoRouteTree::SortMode sortMode = CargoRouteTree::SortMode::amountWaiting;
            std::set<CargoDist::VehicleCargoKey> expandedCompartments;
            std::map<CargoDist::VehicleCargoKey, std::set<CargoRouteTree::GroupKey>> expandedGroups;
            uint64_t cargoRevision = std::numeric_limits<uint64_t>::max();
            uint64_t routingRevision = std::numeric_limits<uint64_t>::max();
            std::map<CargoDist::VehicleCargoKey, std::vector<CargoDist::CargoRouteNode>> routeTrees;
        };

        enum class CargoRowType : uint8_t
        {
            compartment,
            group,
            groupsOmitted,
            empty,
        };

        struct CargoListRow
        {
            CargoRowType type{};
            int32_t y{};
            CargoDist::VehicleCargoKey cargoKey{};
            Vehicles::VehicleCargo cargo{};
            bool expandable{};
            bool expanded{};
            CargoRouteTree::Row group{};
            size_t groupsOmitted{};
        };

        struct CargoCarRow
        {
            Vehicles::Car car{};
            int32_t y{};
            int32_t height{};
        };

        struct CargoList
        {
            std::vector<CargoCarRow> cars;
            std::vector<CargoListRow> rows;
            int32_t height{};
        };

        static std::map<EntityId, CargoWindowState> _windowStates;

        static void onRefitButton(Window& self, const WidgetIndex_t wi, const WidgetId id);

        static CargoWindowState& getWindowState(const EntityId vehicle)
        {
            auto& state = _windowStates[vehicle];
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

        static void eraseWindowState(const EntityId vehicle)
        {
            _windowStates.erase(vehicle);
        }

        static std::vector<CargoDist::CargoRouteNode>& getRouteTree(CargoWindowState& state, const CargoDist::VehicleCargoKey key, const CargoDist::PacketList& packets)
        {
            auto found = state.routeTrees.find(key);
            if (found == state.routeTrees.end())
            {
                const auto summaries = CargoDist::getRouteSummaries(packets);
                found = state.routeTrees.emplace(key, CargoDist::getRouteTree(summaries, CargoRouteTree::getOrder(state.groupOrder))).first;
                CargoRouteTree::sortTree(found->second, state.sortMode);
            }
            else if (state.sortMode == CargoRouteTree::SortMode::station)
            {
                CargoRouteTree::sortTree(found->second, state.sortMode);
            }
            return found->second;
        }

        static void appendCompartmentRows(CargoList& list, CargoWindowState& state, const CargoDist::VehicleCargoKey key, const Vehicles::VehicleCargo& cargo, int32_t& y, size_t& remainingRouteRows, size_t& remainingExpandedCompartments)
        {
            const auto* packets = CargoDist::getVehicleCargoConst(key);
            const auto expandable = packets != nullptr && !packets->empty();
            const auto expanded = expandable && state.expandedCompartments.contains(key);
            list.rows.push_back({
                .type = CargoRowType::compartment,
                .y = y,
                .cargoKey = key,
                .cargo = cargo,
                .expandable = expandable,
                .expanded = expanded,
            });
            y += CargoRouteTree::kRowHeight;

            if (!expanded)
            {
                return;
            }

            size_t omittedRows = 0;
            std::vector<CargoRouteTree::Row> groupRows;
            const auto maxGroupRows = remainingRouteRows > remainingExpandedCompartments ? remainingRouteRows - remainingExpandedCompartments : 0;
            CargoRouteTree::appendRows(groupRows, getRouteTree(state, key, *packets), state.groupOrder, state.expandedGroups[key], maxGroupRows, omittedRows);
            --remainingExpandedCompartments;
            for (const auto& group : groupRows)
            {
                list.rows.push_back({
                    .type = CargoRowType::group,
                    .y = y,
                    .cargoKey = key,
                    .group = group,
                });
                y += CargoRouteTree::kRowHeight;
            }
            remainingRouteRows -= groupRows.size();
            if (omittedRows != 0 && remainingRouteRows != 0)
            {
                list.rows.push_back({
                    .type = CargoRowType::groupsOmitted,
                    .y = y,
                    .cargoKey = key,
                    .groupsOmitted = omittedRows,
                });
                y += CargoRouteTree::kRowHeight;
                --remainingRouteRows;
            }
        }

        static CargoList getCargoList(Window& self)
        {
            CargoList list;
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return list;
            }

            auto& state = getWindowState(head->id);
            Vehicles::Vehicle train{ *head };
            constexpr auto kMaxScrollHeight = static_cast<size_t>(std::numeric_limits<int16_t>::max() - 1);
            const auto baseHeight = std::min(train.cars.size() * self.rowHeight, kMaxScrollHeight);
            auto remainingRouteRows = (kMaxScrollHeight - baseHeight) / CargoRouteTree::kRowHeight;
            size_t remainingExpandedCompartments = 0;
            for (const auto& car : train.cars)
            {
                const auto primaryKey = CargoDist::VehicleCargoKey{ car.body->id, CargoDist::VehicleCargoSlot::primary };
                const auto secondaryKey = CargoDist::VehicleCargoKey{ car.front->id, CargoDist::VehicleCargoSlot::secondary };
                const auto* primaryPackets = CargoDist::getVehicleCargoConst(primaryKey);
                const auto* secondaryPackets = CargoDist::getVehicleCargoConst(secondaryKey);
                remainingExpandedCompartments += car.body->primaryCargo.qty != 0 && primaryPackets != nullptr && !primaryPackets->empty() && state.expandedCompartments.contains(primaryKey);
                remainingExpandedCompartments += car.front->secondaryCargo.qty != 0 && secondaryPackets != nullptr && !secondaryPackets->empty() && state.expandedCompartments.contains(secondaryKey);
            }
            int32_t y = 0;
            for (const auto& car : train.cars)
            {
                const auto primaryKey = CargoDist::VehicleCargoKey{ car.body->id, CargoDist::VehicleCargoSlot::primary };
                const auto secondaryKey = CargoDist::VehicleCargoKey{ car.front->id, CargoDist::VehicleCargoSlot::secondary };
                const auto hasPrimary = car.body->primaryCargo.qty != 0;
                const auto hasSecondary = car.front->secondaryCargo.qty != 0;
                const auto* primaryPackets = CargoDist::getVehicleCargoConst(primaryKey);
                const auto* secondaryPackets = CargoDist::getVehicleCargoConst(secondaryKey);
                const auto primaryExpanded = hasPrimary && primaryPackets != nullptr && !primaryPackets->empty() && state.expandedCompartments.contains(primaryKey);
                const auto secondaryExpanded = hasSecondary && secondaryPackets != nullptr && !secondaryPackets->empty() && state.expandedCompartments.contains(secondaryKey);
                const auto carTop = y;

                if (hasPrimary != hasSecondary && !primaryExpanded && !secondaryExpanded)
                {
                    y += 5;
                }
                if (hasPrimary)
                {
                    appendCompartmentRows(list, state, primaryKey, car.body->primaryCargo, y, remainingRouteRows, remainingExpandedCompartments);
                }
                if (hasSecondary)
                {
                    appendCompartmentRows(list, state, secondaryKey, car.front->secondaryCargo, y, remainingRouteRows, remainingExpandedCompartments);
                }
                if (!hasPrimary && !hasSecondary && car.body->primaryCargo.type != 0xFF)
                {
                    list.rows.push_back({
                        .type = CargoRowType::empty,
                        .y = y + 5,
                    });
                    y += 15;
                }

                const auto carHeight = std::max<int32_t>(self.rowHeight, y - carTop);
                list.cars.push_back({ car, carTop, carHeight });
                y = carTop + carHeight;
            }
            list.height = y;
            return list;
        }

        static const CargoListRow* getRowAt(const CargoList& list, const int16_t y)
        {
            const auto found = std::find_if(list.rows.begin(), list.rows.end(), [y](const auto& row) { return y >= row.y && y < row.y + CargoRouteTree::kRowHeight; });
            return found == list.rows.end() ? nullptr : &*found;
        }

        static const CargoCarRow* getCarAt(const CargoList& list, const int32_t y)
        {
            const auto found = std::find_if(list.cars.begin(), list.cars.end(), [y](const auto& row) { return y >= row.y && y < row.y + row.height; });
            return found == list.cars.end() ? nullptr : &*found;
        }

        static bool canRefit(Vehicles::VehicleHead* headVehicle)
        {
            if (!CompanyManager::isPlayerCompany(headVehicle->owner))
            {
                return false;
            }

            OpenLoco::Vehicles::Vehicle train(*headVehicle);

            if (train.cars.empty())
            {
                return false;
            }

            auto object = ObjectManager::get<VehicleObject>(train.cars.firstCar.front->objectId);
            return (object->hasFlags(VehicleObjectFlags::refittable));
        }

        static void resizeDropdown(Window& self, const widx comboIndex, const widx buttonIndex, const int16_t right)
        {
            auto& combo = self.widgets[comboIndex];
            combo.right = right;
            auto& button = self.widgets[buttonIndex];
            button.right = combo.right - 1;
            button.left = button.right - 10;
        }

        // 0x004B3DDE
        static void prepareDraw(Window& self)
        {
            Common::setActiveTabs(self);
            Common::setCaptionEnableState(self);

            auto* headVehicle = Common::getVehicle(self);
            if (headVehicle == nullptr)
            {
                return;
            }

            // Set title.
            {
                auto args = FormatArguments(self.widgets[Common::widx::caption].textArgs);
                args.push(headVehicle->name);
                args.push(headVehicle->ordinalNumber);
            }

            self.widgets[Common::widx::frame].right = self.width - 1;
            self.widgets[Common::widx::frame].bottom = self.height - 1;
            self.widgets[Common::widx::panel].right = self.width - 1;
            self.widgets[Common::widx::panel].bottom = self.height - 1;
            self.widgets[Common::widx::caption].right = self.width - 2;
            self.widgets[Common::widx::closeButton].left = self.width - 15;
            self.widgets[Common::widx::closeButton].right = self.width - 3;
            resizeDropdown(self, widx::groupBy, widx::groupByDropdown, self.width - 27);
            resizeDropdown(self, widx::sortBy, widx::sortByDropdown, self.width - 27);
            self.widgets[widx::cargoList].right = self.width - 26;
            self.widgets[widx::cargoList].bottom = self.height - 27;
            self.widgets[widx::refit].right = self.width - 2;
            self.widgets[widx::refit].left = self.width - 25;
            self.widgets[widx::refit].hidden = false;
            if (!canRefit(headVehicle))
            {
                self.widgets[widx::refit].hidden = true;
                self.widgets[widx::cargoList].right = self.width - 26 + 22;
                resizeDropdown(self, widx::groupBy, widx::groupByDropdown, self.width - 4);
                resizeDropdown(self, widx::sortBy, widx::sortByDropdown, self.width - 4);
            }

            Widget::leftAlignTabs(self, Common::widx::tabMain, Common::widx::tabRoute);

            const auto& state = getWindowState(headVehicle->id);
            {
                auto& widget = self.widgets[widx::groupBy];
                FormatArguments args{ widget.textArgs };
                args.push(CargoRouteTree::getGroupOrderNames()[static_cast<size_t>(state.groupOrder)]);
            }
            {
                auto& widget = self.widgets[widx::sortBy];
                FormatArguments args{ widget.textArgs };
                args.push(CargoRouteTree::getSortModeNames()[static_cast<size_t>(state.sortMode)]);
            }
        }

        // 0x004B3F0D
        static void draw(Ui::Window& self, Gfx::DrawingContext& drawingCtx)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);

            // draw total cargo
            char* buffer = const_cast<char*>(StringManager::getString(StringIds::buffer_1250));
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            head->generateCargoTotalString(buffer);

            {
                FormatArguments args = {};
                args.push<StringId>(StringIds::buffer_1250);

                auto point = Point(3, self.height - 25);
                tr.drawStringLeftClipped(point, self.width - 15, Colour::black, StringIds::total_stringid, args);
            }

            // draw cargo capacity
            {
                buffer = const_cast<char*>(StringManager::getString(StringIds::buffer_1250));
                head->generateCargoCapacityString(buffer);

                FormatArguments args = {};
                args.push<StringId>(StringIds::buffer_1250);

                auto point = Point(3, self.height - 13);
                tr.drawStringLeftClipped(point, self.width - 15, Colour::black, StringIds::vehicle_capacity_stringid, args);
            }
        }

        // based on 0x004B40C7
        static void drawCargoText(Gfx::DrawingContext& drawingCtx, const int16_t x, int16_t& y, const StringId strFormat, uint8_t cargoQty, uint8_t cargoType, StationId stationId)
        {
            if (cargoQty == 0)
            {
                return;
            }

            auto tr = Gfx::TextRenderer(drawingCtx);

            auto cargoObj = ObjectManager::get<CargoObject>(cargoType);
            auto unitNameFormat = cargoQty == 1 ? cargoObj->unitNameSingular : cargoObj->unitNamePlural;
            auto station = StationManager::get(stationId);

            FormatArguments args{};
            args.push(StringIds::cargo_from);
            args.push(unitNameFormat);
            args.push<uint32_t>(cargoQty);
            args.push(station->name);
            args.push(station->town);

            auto point = Point(x, y);
            tr.drawStringLeft(point, Colour::black, strFormat, args);
            y += 10;
        }

        static void drawCargoRow(Gfx::DrawingContext& drawingCtx, const CargoListRow& row, const StringId strFormat)
        {
            constexpr int16_t kCargoXPos = 24;
            if (row.expandable)
            {
                CargoRouteTree::drawDisclosure(drawingCtx, kCargoXPos + 2, static_cast<int16_t>(row.y + 5), row.expanded);
            }
            auto y = static_cast<int16_t>(row.y + 1);
            drawCargoText(drawingCtx, row.expandable ? kCargoXPos + 10 : kCargoXPos, y, strFormat, row.cargo.qty, row.cargo.type, row.cargo.townFrom);
        }

        static void drawGroupsOmitted(Window& self, Gfx::DrawingContext& drawingCtx, const CargoListRow& row)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);
            FormatArguments args{};
            args.push<int32_t>(static_cast<int32_t>(std::min<size_t>(row.groupsOmitted, std::numeric_limits<int32_t>::max())));
            const auto width = std::max<int32_t>(self.widgets[widx::cargoList].width() - 46, 0);
            tr.drawStringLeftClipped({ 34, static_cast<int16_t>(row.y + 1) }, width, Colour::black, StringIds::station_cargo_groups_omitted, args);
        }

        // 0x004B3F62
        static void drawScroll(Window& self, Gfx::DrawingContext& drawingCtx, [[maybe_unused]] const uint32_t i)
        {
            drawingCtx.clearSingle(Colours::getShade(self.getColour(WindowColour::secondary).c(), 4));
            const auto list = getCargoList(self);
            const auto& renderTarget = drawingCtx.currentRenderTarget();
            constexpr int16_t kCargoXPos = 24;
            for (const auto& carRow : list.cars)
            {
                if (carRow.y + carRow.height < renderTarget.y || carRow.y >= renderTarget.y + renderTarget.height)
                {
                    continue;
                }

                if (carRow.car.front->id == EntityId(self.rowHover))
                {
                    drawingCtx.fillRect(0, carRow.y, kCargoXPos - 1, carRow.y + self.rowHeight - 1, enumValue(ExtColour::unk30), Gfx::RectFlags::transparent);
                }

                const auto width = getWidthVehicleInline(carRow.car);
                const auto vehicleY = carRow.y + (self.rowHeight - 22) / 2;
                drawVehicleInline(drawingCtx, carRow.car, Ui::Point(kCargoXPos - width, vehicleY), VehicleInlineMode::basic, VehiclePartsToDraw::bogies);
                drawVehicleInline(drawingCtx, carRow.car, Ui::Point(kCargoXPos - width, vehicleY), VehicleInlineMode::basic, VehiclePartsToDraw::bodies);
            }

            auto tr = Gfx::TextRenderer(drawingCtx);
            for (const auto& row : list.rows)
            {
                if (row.y + CargoRouteTree::kRowHeight < renderTarget.y)
                {
                    continue;
                }
                if (row.y >= renderTarget.y + renderTarget.height)
                {
                    break;
                }

                const auto* car = getCarAt(list, row.y);
                const auto isHovered = car != nullptr && car->car.front->id == EntityId(self.rowHover);
                const auto strFormat = isHovered ? StringIds::wcolour2_stringid : StringIds::black_stringid;
                if (isHovered && (row.type == CargoRowType::compartment || row.type == CargoRowType::empty))
                {
                    drawingCtx.fillRect(kCargoXPos, row.y, self.widgets[widx::cargoList].width(), row.y + CargoRouteTree::kRowHeight - 1, enumValue(ExtColour::unk30), Gfx::RectFlags::transparent);
                }
                switch (row.type)
                {
                    case CargoRowType::compartment:
                        drawCargoRow(drawingCtx, row, strFormat);
                        break;
                    case CargoRowType::group:
                        CargoRouteTree::drawRow(drawingCtx, row.group, row.y, self.widgets[widx::cargoList].width(), kCargoXPos);
                        break;
                    case CargoRowType::groupsOmitted:
                        drawGroupsOmitted(self, drawingCtx, row);
                        break;
                    case CargoRowType::empty:
                    {
                        FormatArguments args{};
                        args.push<StringId>(StringIds::cargo_empty);
                        tr.drawStringLeft({ kCargoXPos, static_cast<int16_t>(row.y + 1) }, Colour::black, strFormat, args);
                        break;
                    }
                }
            }
        }

        static void updateScrollSize(Window& self, const uint8_t scrollIndex)
        {
            self.updateScrollWidgets();
            auto& scrollArea = self.scrollAreas[scrollIndex];
            const auto visibleHeight = self.widgets[widx::cargoList].height() - 2;
            const auto maxOffset = std::max<int32_t>(scrollArea.contentHeight - visibleHeight, 0);
            scrollArea.contentOffsetY = std::clamp<int32_t>(scrollArea.contentOffsetY, 0, maxOffset);
            ScrollView::updateThumbs(self, widx::cargoList);
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

        static void expandAll(Window& self)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }

            auto& state = getWindowState(head->id);
            Vehicles::Vehicle train{ *head };
            for (const auto& car : train.cars)
            {
                for (const auto key : {
                    CargoDist::VehicleCargoKey{ car.body->id, CargoDist::VehicleCargoSlot::primary },
                    CargoDist::VehicleCargoKey{ car.front->id, CargoDist::VehicleCargoSlot::secondary } })
                {
                    const auto* packets = CargoDist::getVehicleCargoConst(key);
                    if (packets == nullptr || packets->empty())
                    {
                        continue;
                    }
                    state.expandedCompartments.insert(key);
                    CargoRouteTree::expandAllGroups(state.expandedGroups[key], getRouteTree(state, key, *packets));
                }
            }
            updateScrollSize(self, 0);
            self.invalidate();
        }

        // 0x004B41BD
        static void onMouseUp(Window& self, const WidgetIndex_t i, const WidgetId id)
        {
            switch (id)
            {
                case Common::Widx::kCloseButton:
                    WindowManager::close(&self);
                    break;

                case Common::Widx::kTabMain:
                case Common::Widx::kTabDetails:
                case Common::Widx::kTabCargo:
                case Common::Widx::kTabFinances:
                case Common::Widx::kTabRoute:
                    Common::switchTab(self, i);
                    break;

                case Common::Widx::kCaption:
                    Common::renameVehicle(self, i);
                    break;

                case Widx::kExpandAll:
                    expandAll(self);
                    break;
            }
        }

        // 0x004B41E2
        static void onMouseDown(Window& self, const WidgetIndex_t i, const WidgetId id)
        {
            const auto& state = getWindowState(EntityId(self.number));
            if (id == Widx::kGroupByDropdown)
            {
                showDropdown(self, widx::groupBy, CargoRouteTree::getGroupOrderNames(), static_cast<size_t>(state.groupOrder));
            }
            else if (id == Widx::kSortByDropdown)
            {
                showDropdown(self, widx::sortBy, CargoRouteTree::getSortModeNames(), static_cast<size_t>(state.sortMode));
            }
            else if (id == Widx::kRefit)
            {
                onRefitButton(self, i, id);
            }
        }

        // 0x004B41E9
        static void onDropdown(Window& self, [[maybe_unused]] const WidgetIndex_t i, const WidgetId id, const int16_t dropdownIndex)
        {
            if (dropdownIndex < 0)
            {
                return;
            }

            auto& state = getWindowState(EntityId(self.number));
            if (id == Widx::kGroupByDropdown && static_cast<size_t>(dropdownIndex) < CargoRouteTree::getGroupOrderNames().size())
            {
                const auto groupOrder = static_cast<CargoRouteTree::GroupOrder>(dropdownIndex);
                if (state.groupOrder == groupOrder)
                {
                    return;
                }
                state.groupOrder = groupOrder;
                state.expandedGroups.clear();
                state.routeTrees.clear();
            }
            else if (id == Widx::kSortByDropdown && static_cast<size_t>(dropdownIndex) < CargoRouteTree::getSortModeNames().size())
            {
                const auto sortMode = static_cast<CargoRouteTree::SortMode>(dropdownIndex);
                if (state.sortMode == sortMode)
                {
                    return;
                }
                state.sortMode = sortMode;
                state.routeTrees.clear();
            }
            else if (id == Widx::kRefit)
            {
                GameCommands::VehicleRefitArgs args{};
                args.head = static_cast<EntityId>(self.number);
                args.cargoType = Dropdown::getItemArgument(dropdownIndex, 3);

                if (Common::confirmComponentChange(args.head, StringIds::confirm_vehicle_component_refit_cargo_warning_title, StringIds::confirm_vehicle_component_refit_cargo_warning_message, StringIds::confirm_vehicle_component_refit_cargo_warning_confirm, false))
                {
                    GameCommands::setErrorTitle(StringIds::cant_refit_vehicle);
                    GameCommands::doCommand(args, GameCommands::Flags::apply);
                }
                return;
            }
            else
            {
                return;
            }

            updateScrollSize(self, 0);
            self.invalidate();
        }

        static void onRefitButton(Window& self, const WidgetIndex_t wi, [[maybe_unused]] const WidgetId id)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            Vehicles::Vehicle train(*head);
            auto vehicleObject = ObjectManager::get<VehicleObject>(train.cars.firstCar.front->objectId);
            auto maxPrimaryCargo = vehicleObject->maxCargo[0];
            auto primaryCargoId = Numerics::bitScanForward(vehicleObject->compatibleCargoCategories[0]);

            int32_t index = 0;
            for (uint16_t cargoId = 0; cargoId < ObjectManager::getMaxObjects(ObjectType::cargo); cargoId++)
            {
                auto cargoObject = ObjectManager::get<CargoObject>(cargoId);
                if (cargoObject == nullptr)
                {
                    continue;
                }

                if (!cargoObject->hasFlags(CargoObjectFlags::refit))
                {
                    continue;
                }

                StringId format = StringIds::dropdown_stringid;
                if (cargoId == train.cars.firstCar.body->primaryCargo.type)
                {
                    format = StringIds::dropdown_stringid_selected;
                }

                FormatArguments args{};
                args.push<StringId>(cargoObject->unitNamePlural);
                args.push<uint32_t>(Vehicles::getNumUnitsForCargo(maxPrimaryCargo, primaryCargoId, cargoId));
                args.push<uint16_t>(cargoId);
                Dropdown::add(index, format, args);
                index++;
            }

            Widget& button = self.widgets[wi];

            Dropdown::showText(
                self.x + button.left,
                self.y + button.top,
                button.width(),
                button.height(),
                self.getColour(WindowColour::secondary),
                index,
                0);
            Dropdown::setHighlightedItem(0);
        }

        // 0x004B4339
        static std::optional<FormatArguments> tooltip(Ui::Window& self, WidgetIndex_t, [[maybe_unused]] const WidgetId id)
        {
            FormatArguments args{};
            args.push(StringIds::tooltip_scroll_vehicle_list);

            auto vehicle = Common::getVehicle(self);
            if (vehicle == nullptr)
            {
                return {};
            }
            args.push(StringIds::getVehicleType(vehicle->vehicleType));
            return args;
        }

        // 0x004B4360
        static void getScrollSize(Ui::Window& self, [[maybe_unused]] const uint32_t scrollIndex, [[maybe_unused]] int32_t& scrollWidth, int32_t& scrollHeight)
        {
            scrollHeight = getCargoList(self).height;
        }

        static char* generateCargoTooltipDetails(char* buffer, const StringId cargoFormat, const uint8_t cargoType, const uint8_t maxCargo, const uint32_t acceptedCargoTypes)
        {
            if (cargoType == 0xFF)
            {
                return buffer;
            }

            {
                auto cargoObj = ObjectManager::get<CargoObject>(cargoType);
                FormatArguments args{};
                args.push(maxCargo == 1 ? cargoObj->unitNameSingular : cargoObj->unitNamePlural);
                args.push<uint32_t>(maxCargo);
                buffer = StringManager::formatString(buffer, cargoFormat, args);
            }

            auto availableCargoTypes = acceptedCargoTypes & ~(1 << cargoType);
            if (availableCargoTypes != 0)
            {
                *buffer++ = ' ';
                *buffer++ = '(';

                while (availableCargoTypes != 0)
                {
                    auto type = Numerics::bitScanForward(availableCargoTypes);
                    availableCargoTypes &= ~(1 << type);

                    auto cargoObj = ObjectManager::get<CargoObject>(type);
                    FormatArguments args{};
                    args.push(cargoObj->name);
                    buffer = StringManager::formatString(buffer, StringIds::stats_or_string, args);
                    *buffer++ = ' ';
                }
                --buffer;
                *buffer++ = ')';
                *buffer++ = '\0';
            }
            return buffer;
        }

        // 0x004B4404
        static void scrollMouseOver(Window& self, [[maybe_unused]] const int16_t x, const int16_t y, [[maybe_unused]] const uint8_t scrollIndex)
        {
            Ui::ToolTip::setTooltipTimeout(2000);
            self.flags &= ~WindowFlags::notScrollView;
            const auto list = getCargoList(self);
            const auto* carRow = getCarAt(list, y);
            StringId tooltipFormat = StringIds::null;
            EntityId tooltipContent = EntityId::null;
            if (carRow != nullptr)
            {
                tooltipFormat = StringIds::buffer_337;
                tooltipContent = carRow->car.front->id;
                if (EntityId(self.rowHover) != tooltipContent)
                {
                    self.rowHover = enumValue(tooltipContent);
                    self.invalidate();
                }
            }
            else if (self.rowHover != -1)
            {
                self.rowHover = -1;
                self.invalidate();
            }

            char* buffer = const_cast<char*>(StringManager::getString(StringIds::buffer_337));
            if (*buffer != '\0')
            {
                if (self.widgets[widx::cargoList].tooltip == tooltipFormat && EntityId(self.var_85C) == tooltipContent)
                {
                    return;
                }
            }

            self.widgets[widx::cargoList].tooltip = tooltipFormat;
            self.var_85C = enumValue(tooltipContent);
            ToolTip::closeAndReset();

            if (tooltipContent == EntityId::null)
            {
                return;
            }

            Ui::ToolTip::set_52336E(true);

            {
                auto vehicleObj = ObjectManager::get<VehicleObject>(carRow->car.front->objectId);
                FormatArguments args{};
                args.push(vehicleObj->name);
                buffer = StringManager::formatString(buffer, StringIds::cargo_capacity_tooltip, args);
            }

            auto body = carRow->car.body;
            auto front = carRow->car.front;
            buffer = generateCargoTooltipDetails(buffer, StringIds::cargo_capacity, body->primaryCargo.type, body->primaryCargo.maxQty, body->primaryCargo.acceptedTypes);
            buffer = generateCargoTooltipDetails(buffer, StringIds::cargo_capacity_plus, front->secondaryCargo.type, front->secondaryCargo.maxQty, front->secondaryCargo.acceptedTypes);
        }

        static bool isDisclosureHit(const CargoListRow& row, const int16_t x)
        {
            if (row.type == CargoRowType::compartment)
            {
                return row.expandable && x >= 26 && x <= 32;
            }
            return row.type == CargoRowType::group && CargoRouteTree::isDisclosureHit(row.group, x, 24);
        }

        static bool isStationLinkHit(Window& self, const CargoListRow& row, const int16_t x)
        {
            return row.type == CargoRowType::group && CargoRouteTree::isStationLinkHit(row.group, x, self.widgets[widx::cargoList].width(), 24);
        }

        static void onScrollMouseDown(Window& self, const int16_t x, const int16_t y, const uint8_t scrollIndex)
        {
            const auto list = getCargoList(self);
            const auto* row = getRowAt(list, y);
            if (row == nullptr)
            {
                return;
            }

            if (isDisclosureHit(*row, x))
            {
                auto& state = getWindowState(EntityId(self.number));
                if (row->type == CargoRowType::compartment)
                {
                    if (state.expandedCompartments.erase(row->cargoKey) == 0)
                    {
                        state.expandedCompartments.insert(row->cargoKey);
                    }
                }
                else
                {
                    auto& expandedGroups = state.expandedGroups[row->cargoKey];
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
            if (id != Widx::kCargoList)
            {
                return fallback;
            }

            const auto list = getCargoList(self);
            const auto* row = getRowAt(list, y);
            return row != nullptr && (isDisclosureHit(*row, x) || isStationLinkHit(self, *row, x)) ? CursorId::handPointer : fallback;
        }

        // 0x004B4607
        static void onUpdate(Window& self)
        {
            self.frameNo += 1;
            self.callPrepareDraw();
            updateScrollSize(self, 0);
            self.invalidate();
        }

        static constexpr WindowEventList kEvents = {
            .onClose = Common::onClose,
            .onMouseUp = onMouseUp,
            .onMouseDown = onMouseDown,
            .onDropdown = onDropdown,
            .onUpdate = onUpdate,
            .onHandleInputBegin = listWindowOnHandleInputBegin,
            .onHandleInputEnd = listWindowOnHandleInputEnd,
            .getScrollSize = getScrollSize,
            .scrollMouseDown = onScrollMouseDown,
            .scrollMouseOver = scrollMouseOver,
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

    namespace Finances
    {
        // 0x004B56CE
        static void prepareDraw(Window& self)
        {
            Common::setActiveTabs(self);
            Common::setCaptionEnableState(self);

            auto vehicle = Common::getVehicle(self);
            if (vehicle == nullptr)
            {
                return;
            }

            // Set title.
            {
                auto args = FormatArguments(self.widgets[Common::widx::caption].textArgs);
                args.push(vehicle->name);
                args.push(vehicle->ordinalNumber);
            }

            self.widgets[Common::widx::frame].right = self.width - 1;
            self.widgets[Common::widx::frame].bottom = self.height - 1;
            self.widgets[Common::widx::panel].right = self.width - 1;
            self.widgets[Common::widx::panel].bottom = self.height - 1;
            self.widgets[Common::widx::caption].right = self.width - 2;
            self.widgets[Common::widx::closeButton].left = self.width - 15;
            self.widgets[Common::widx::closeButton].right = self.width - 3;

            Widget::leftAlignTabs(self, Common::widx::tabMain, Common::widx::tabRoute);
        }

        // 0x004B576C
        static void draw(Ui::Window& self, Gfx::DrawingContext& drawingCtx)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);

            auto pos = Ui::Point(4, 46);

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            Vehicles::Vehicle train(*head);
            auto veh1 = train.veh1;
            if (veh1->lastIncome.day != -1)
            {
                {
                    FormatArguments args{};
                    args.push<uint32_t>(veh1->lastIncome.day);
                    // Last income on: {DATE DMY}
                    tr.drawStringLeft(pos, Colour::black, StringIds::last_income_on_date, args);
                }

                pos.y += 10;
                for (int i = 0; i < 4; i++)
                {
                    auto cargoType = veh1->lastIncome.cargoTypes[i];
                    if (cargoType == 0xFF)
                    {
                        continue;
                    }

                    auto cargoObject = ObjectManager::get<CargoObject>(cargoType);

                    auto str = veh1->lastIncome.cargoQtys[i] == 1 ? cargoObject->unitNameSingular : cargoObject->unitNamePlural;

                    FormatArguments args{};
                    args.push(str);
                    args.push<uint32_t>(veh1->lastIncome.cargoQtys[i]);
                    args.push(veh1->lastIncome.cargoDistances[i]);
                    args.push<uint16_t>(veh1->lastIncome.cargoAges[i]);
                    args.push<currency32_t>(veh1->lastIncome.cargoProfits[i]);

                    // {STRINGID} transported {INT16} blocks in {INT16} days = {CURRENCY32}
                    pos.x += 4;
                    tr.drawStringLeftWrapped(pos, self.width - 12, Colour::black, StringIds::transported_blocks_in_days, args);
                    pos.x -= 4;

                    // TODO: fix function to take pointer to offset
                    pos.y += 12;
                }
            }
            else
            {
                // Last income: N/A"
                tr.drawStringLeft(pos, Colour::black, StringIds::last_income_na);
                pos.y += 10;
            }

            pos.y += 5;

            if (head->lastAverageSpeed != 0_mph)
            {
                // Last journey average speed: {VELOCITY}
                FormatArguments args{};
                args.setTransportMode(enumValue(head->mode));
                args.push(head->lastAverageSpeed);
                tr.drawStringLeft(pos, Colour::black, StringIds::last_journey_average_speed, args);
                pos.y += 10 + 5;
            }

            {
                // Monthly Running Cost: {CURRENCY32}
                FormatArguments args{};
                args.push(head->calculateRunningCost());
                tr.drawStringLeft(pos, Colour::black, StringIds::vehicle_monthly_running_cost, args);
                pos.y += 10;
            }

            {
                // Monthly Profit: {CURRENCY32}
                FormatArguments args{};
                auto monthlyProfit = (train.veh2->totalRecentProfit()) / 4;
                args.push(monthlyProfit);
                tr.drawStringLeft(pos, Colour::black, StringIds::vehicle_monthly_profit, args);
                pos.y += 10 + 5;
            }

            {
                // Sale value of vehicle: {CURRENCY32}
                FormatArguments args{};
                args.push(train.head->totalRefundCost);
                pos.y = self.height - 14;
                tr.drawStringLeft(pos, Colour::black, StringIds::sale_value_of_vehicle, args);
            }
        }

        // 0x004B5945
        static void onMouseUp(Window& self, WidgetIndex_t widgetIndex, const WidgetId id)
        {
            switch (id)
            {
                case Common::Widx::kCloseButton:
                    WindowManager::close(&self);
                    break;
                case Common::Widx::kCaption:
                    Common::renameVehicle(self, widgetIndex);
                    break;
                case Common::Widx::kTabMain:
                case Common::Widx::kTabDetails:
                case Common::Widx::kTabCargo:
                case Common::Widx::kTabFinances:
                case Common::Widx::kTabRoute:
                    Common::switchTab(self, widgetIndex);
                    break;
            }
        }

        // 0x004B5977
        static std::optional<FormatArguments> tooltip(Ui::Window& self, WidgetIndex_t, [[maybe_unused]] const WidgetId id)
        {
            FormatArguments args{};
            auto veh0 = Common::getVehicle(self);
            if (veh0 == nullptr)
            {
                return {};
            }
            args.skip(2);
            args.push(StringIds::getVehicleType(veh0->vehicleType));
            return args;
        }

        // 0x004B5995
        static void onUpdate(Window& self)
        {
            self.frameNo += 1;
            self.callPrepareDraw();
            WindowManager::invalidateWidget(self.type, self.number, Common::widx::tabFinances);
        }

        static constexpr WindowEventList kEvents = {
            .onClose = Common::onClose,
            .onMouseUp = onMouseUp,
            .onUpdate = onUpdate,
            .textInput = Common::textInput,
            .tooltip = tooltip,
            .prepareDraw = prepareDraw,
            .draw = draw,
        };

        static const WindowEventList& getEvents()
        {
            return kEvents;
        }
    }

    namespace Route
    {
        enum class SharedOrderDropdownMode
        {
            none,
            joinSource,
            members,
        };

        static constexpr size_t kVehicleDropdownPageSize = 30;
        static std::vector<EntityId> _sharedOrderDropdownVehicles;
        static size_t _sharedOrderDropdownPageStart;
        static EntityId _sharedOrderDropdownTarget = EntityId::null;
        static SharedOrderDropdownMode _sharedOrderDropdownMode = SharedOrderDropdownMode::none;
        static constexpr size_t kTimetableSlotPageSize = 30;
        static std::vector<uint32_t> _timetableDropdownSlots;
        static std::vector<std::string> _timetableDropdownSlotLabels;
        static size_t _timetableSlotPageStart;
        struct TimetableTarget
        {
            EntityId vehicle = EntityId::null;
            Vehicles::TimetableManager::ServiceId service = Vehicles::TimetableManager::kInvalidServiceId;
            Vehicles::TimetableManager::EntryId entry = Vehicles::TimetableManager::kInvalidEntryId;
            uint32_t serviceRevision{};
            bool evenlySpacedSlots{};
        };
        static TimetableTarget _timetableDropdownTarget;
        static TimetableTarget _timetableInputTarget;
        static std::map<EntityId, Vehicles::TimetableManager::EntryId> _timetableSelections;
        static std::map<EntityId, std::array<std::string, 6>> _timetableDisplayTimes;

        static std::string formatTimetableDuration(const uint64_t minutes)
        {
            return fmt::format("{:02}:{:02}", minutes / 60, minutes % 60);
        }

        static std::string formatTimetableClockTime(const uint64_t minutes)
        {
            return formatTimetableDuration(minutes % (24 * 60));
        }

        static void resetSharedOrderDropdown(const EntityId target)
        {
            if (_sharedOrderDropdownTarget == target)
            {
                _sharedOrderDropdownVehicles.clear();
                _sharedOrderDropdownPageStart = 0;
                _sharedOrderDropdownTarget = EntityId::null;
                _sharedOrderDropdownMode = SharedOrderDropdownMode::none;
            }
            if (_timetableDropdownTarget.vehicle == target)
            {
                _timetableDropdownSlots.clear();
                _timetableDropdownSlotLabels.clear();
                _timetableSlotPageStart = 0;
                _timetableDropdownTarget = {};
            }
            if (_timetableInputTarget.vehicle == target)
            {
                _timetableInputTarget = {};
            }
            _timetableSelections.erase(target);
            _timetableDisplayTimes.erase(target);
        }

        static Vehicles::OrderRingView getOrderTable(const Vehicles::VehicleHead* const head)
        {
            return Vehicles::OrderRingView(head->orderTableOffset);
        }

        // 0x004B509B
        static void close(Window& self)
        {
            Cargo::eraseWindowState(EntityId(self.number));
            resetSharedOrderDropdown(EntityId(self.number));
            if (ToolManager::isToolActive(self.type, self.number))
            {
                ToolManager::toolCancel();
            }
        }

        static void orderDeleteCommand(Vehicles::VehicleHead* const head, const uint32_t orderOffset)
        {
            GameCommands::VehicleOrderDeleteArgs args{};
            args.head = head->id;
            args.orderOffset = orderOffset - head->orderTableOffset;

            GameCommands::setErrorTitle(StringIds::empty);
            GameCommands::doCommand(args, GameCommands::Flags::apply);

            Vehicles::OrderManager::generateNumDisplayFrames(head);
        }

        // 0x004B4F6D
        static void onOrderDelete(Vehicles::VehicleHead* const head, const int16_t orderId)
        {
            // No deletable orders
            if (head->sizeOfOrderTable <= 1)
            {
                return;
            }

            // orderId can be -1 at this point for none selected
            auto i = 0;
            const Vehicles::Order* last = nullptr;
            for (const auto& order : getOrderTable(head))
            {
                if (i == orderId)
                {
                    orderDeleteCommand(head, order.getOffset());
                    return;
                }
                last = &order;
                i++;
            }
            // No order selected so delete the last one.
            if (orderId == -1 && last != nullptr)
            {
                orderDeleteCommand(head, last->getOffset());
            }
        }

        // 0x004B4C14
        static bool orderUpCommand(Vehicles::VehicleHead* const head, const uint32_t orderOffset)
        {
            GameCommands::VehicleOrderUpArgs args{};
            args.head = head->id;
            args.orderOffset = orderOffset - head->orderTableOffset;

            GameCommands::setErrorTitle(StringIds::empty);
            auto result = GameCommands::doCommand(args, GameCommands::Flags::apply);

            Vehicles::OrderManager::generateNumDisplayFrames(head); // Note: order changed, check if this matters.
            return result != GameCommands::kFailure;
        }

        // 0x004B4CCB based on
        static bool orderDownCommand(Vehicles::VehicleHead* const head, const uint32_t orderOffset)
        {
            GameCommands::VehicleOrderDownArgs args{};
            args.head = head->id;
            args.orderOffset = orderOffset - head->orderTableOffset;

            GameCommands::setErrorTitle(StringIds::empty);
            auto result = GameCommands::doCommand(args, GameCommands::Flags::apply);

            Vehicles::OrderManager::generateNumDisplayFrames(head); // Note: order changed, check if this matters.
            return result != GameCommands::kFailure;
        }

        static bool orderReverseCommand(Vehicles::VehicleHead* const head)
        {
            GameCommands::VehicleOrderReverseArgs args{};
            args.head = head->id;

            GameCommands::setErrorTitle(StringIds::empty);
            auto result = GameCommands::doCommand(args, GameCommands::Flags::apply);

            Vehicles::OrderManager::generateNumDisplayFrames(head);
            return result != GameCommands::kFailure;
        }

        // 0x004B4BC1 / 0x004B4C78 based on
        static bool onOrderMove(Vehicles::VehicleHead* const head, const int16_t orderId, bool(orderMoveFunc)(Vehicles::VehicleHead*, uint32_t))
        {
            // No moveable orders
            if (head->sizeOfOrderTable <= 1)
            {
                return false;
            }
            // Valid orderId should be positive (avoid -1 / null)
            if (orderId < 0)
            {
                return false;
            }

            auto* order = getOrderTable(head).atIndex(orderId);
            if (order != nullptr)
            {
                return orderMoveFunc(head, order->getOffset());
            }
            return false;
        }

        static Vehicles::TimetableManager::TimetableEntry* getSelectedTimetableEntry(Window& self, const Vehicles::VehicleHead& head)
        {
            if (self.orderTableIndex < 0 || self.orderTableIndex > std::numeric_limits<uint8_t>::max())
            {
                return nullptr;
            }
            return Vehicles::TimetableManager::getEntry(head.id, static_cast<uint8_t>(self.orderTableIndex));
        }

        static Vehicles::TimetableManager::TimetableEntry* findTimetableEntry(
            const Vehicles::TimetableManager::ServiceId serviceId,
            const Vehicles::TimetableManager::EntryId entryId)
        {
            auto* service = Vehicles::TimetableManager::getService(serviceId);
            if (service == nullptr)
            {
                return nullptr;
            }
            const auto entry = std::ranges::find(service->entries, entryId, &Vehicles::TimetableManager::TimetableEntry::id);
            return entry != service->entries.end() ? &*entry : nullptr;
        }

        static bool confirmTimetableAction(const StringId message, const StringId button)
        {
            FormatArguments args{};
            return Windows::PromptOkCancel::open(StringIds::cant_change_timetable, message, args, button);
        }

        static uint32_t timetableCommand(
            const Vehicles::VehicleHead& head,
            const GameCommands::VehicleTimetableArgs::Action action,
            const uint16_t orderIndex,
            const uint32_t value)
        {
            GameCommands::setErrorTitle(StringIds::cant_change_timetable);
            GameCommands::VehicleTimetableArgs args{};
            args.head = head.id;
            args.action = action;
            args.orderIndex = orderIndex;
            args.value = value;
            return GameCommands::doCommand(args, GameCommands::Flags::apply);
        }

        static void openTimetableInput(Window& self, const WidgetIndex_t widgetIndex, const WidgetId id, const bool evenlySpacedSlots = false)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }

            std::optional<uint32_t> value;
            const auto* entry = getSelectedTimetableEntry(self, *head);
            if (id == Widx::kTimetableClockRate)
            {
                value = Vehicles::TimetableManager::getTicksPerMinute();
            }
            else if (entry == nullptr)
            {
                return;
            }
            else if (id == Widx::kTimetableTravel)
            {
                value = entry->travelMinutes;
            }
            else if (id == Widx::kTimetableDwell)
            {
                value = entry->dwellMinutes;
            }
            else if (id == Widx::kTimetablePeriod)
            {
                value = entry->dispatch.has_value() ? entry->dispatch->periodMinutes : Vehicles::TimetableManager::kDefaultPeriodMinutes;
            }
            else if (id == Widx::kTimetablePhase)
            {
                value = entry->dispatch.has_value() ? entry->dispatch->phaseMinutes : 0;
            }
            else if (id == Widx::kTimetableMaxDelay)
            {
                value = entry->dispatch.has_value() ? entry->dispatch->maxDelayMinutes : 0;
            }
            else if (id != Widx::kTimetableAddSlot)
            {
                return;
            }

            const auto serviceId = Vehicles::TimetableManager::getServiceId(head->id);
            const auto* service = Vehicles::TimetableManager::getService(serviceId);
            if (service == nullptr)
            {
                return;
            }
            _timetableInputTarget = { head->id, serviceId, entry != nullptr ? entry->id : Vehicles::TimetableManager::kInvalidEntryId, service->revision, evenlySpacedSlots };

            FormatArguments args{};
            std::string formattedValue;
            const auto valueString = value.has_value()
                ? (id == Widx::kTimetableClockRate ? StringIds::timetable_input_value : StringIds::stringptr)
                : StringIds::empty;
            if (value.has_value())
            {
                if (id == Widx::kTimetableClockRate)
                {
                    args.push<int32_t>(*value);
                }
                else
                {
                    formattedValue = formatTimetableDuration(*value);
                    args.push(formattedValue.c_str());
                }
            }
            const auto prompt = evenlySpacedSlots
                ? StringIds::timetable_even_slots_prompt
                : (id == Widx::kTimetableClockRate ? StringIds::timetable_clock_rate_prompt : StringIds::timetable_edit_prompt);
            TextInput::openTextInput(&self, StringIds::timetable_edit_title, prompt, valueString, widgetIndex, args, 10);
        }

        static void showTimetableAddSlotDropdown(Window& self, const WidgetIndex_t widgetIndex)
        {
            Dropdown::add(0, StringIds::timetable_add_single_slot);
            Dropdown::add(1, StringIds::timetable_set_even_slots);
            Dropdown::showBelow(&self, widgetIndex, 2, 0);
            Dropdown::setHighlightedItem(0);
        }

        static void showTimetableSlotDropdownPage(Window& self, const WidgetIndex_t widgetIndex)
        {
            const auto pageEnd = std::min(_timetableSlotPageStart + kTimetableSlotPageSize, _timetableDropdownSlots.size());
            size_t dropdownIndex = 0;
            for (auto i = _timetableSlotPageStart; i < pageEnd; ++i)
            {
                FormatArguments args{};
                args.push(_timetableDropdownSlotLabels[i].c_str());
                Dropdown::add(dropdownIndex++, StringIds::timetable_slot_value, args);
            }
            if (pageEnd < _timetableDropdownSlots.size() || _timetableSlotPageStart != 0)
            {
                Dropdown::add(dropdownIndex++, pageEnd < _timetableDropdownSlots.size() ? StringIds::timetable_more_slots : StringIds::timetable_first_slots);
            }

            const auto& widget = self.widgets[widgetIndex];
            Dropdown::showText(
                self.x + widget.left,
                self.y + widget.top,
                std::max<int16_t>(widget.width(), 100),
                widget.height(),
                self.getColour(WindowColour::secondary),
                dropdownIndex,
                0);
        }

        static void showTimetableSlotDropdown(Window& self, const WidgetIndex_t widgetIndex)
        {
            auto* head = Common::getVehicle(self);
            auto* entry = head != nullptr ? getSelectedTimetableEntry(self, *head) : nullptr;
            if (head == nullptr || entry == nullptr || !entry->dispatch.has_value() || entry->dispatch->slots.empty())
            {
                return;
            }
            _timetableDropdownSlots = entry->dispatch->slots;
            _timetableDropdownSlotLabels.clear();
            _timetableDropdownSlotLabels.reserve(_timetableDropdownSlots.size());
            for (const auto slot : _timetableDropdownSlots)
            {
                _timetableDropdownSlotLabels.push_back(formatTimetableDuration(slot));
            }
            _timetableSlotPageStart = 0;
            const auto serviceId = Vehicles::TimetableManager::getServiceId(head->id);
            const auto* service = Vehicles::TimetableManager::getService(serviceId);
            if (service == nullptr)
            {
                return;
            }
            _timetableDropdownTarget = { head->id, serviceId, entry->id, service->revision };
            showTimetableSlotDropdownPage(self, widgetIndex);
        }

        static std::optional<uint32_t> parseTimetableValue(const std::string_view text, const bool allowClockFormat)
        {
            const auto parseNumber = [](const std::string_view value) -> std::optional<uint32_t> {
                uint32_t result{};
                const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
                return error == std::errc{} && end == value.data() + value.size() ? std::optional<uint32_t>{ result } : std::nullopt;
            };
            const auto separator = text.find(':');
            if (!allowClockFormat || separator == std::string_view::npos)
            {
                return parseNumber(text);
            }
            if (separator == 0 || separator + 1 == text.size() || text.find(':', separator + 1) != std::string_view::npos)
            {
                return std::nullopt;
            }
            const auto hours = parseNumber(text.substr(0, separator));
            const auto minutes = parseNumber(text.substr(separator + 1));
            if (!hours.has_value() || !minutes.has_value() || *minutes >= 60
                || *hours > (std::numeric_limits<uint32_t>::max() - *minutes) / 60)
            {
                return std::nullopt;
            }
            return *hours * 60 + *minutes;
        }

        static void timetableTextInput(Window& self, const WidgetIndex_t callingWidget, const WidgetId id, const char* const input)
        {
            if (id == Common::Widx::kCaption)
            {
                Common::textInput(self, callingWidget, id, input);
                return;
            }

            const auto inputTarget = _timetableInputTarget;
            _timetableInputTarget = {};
            auto* head = Common::getVehicle(self);
            const auto* service = head != nullptr ? Vehicles::TimetableManager::getServiceForVehicle(head->id) : nullptr;
            if (head == nullptr || inputTarget.vehicle != head->id || service == nullptr
                || inputTarget.service != service->id || inputTarget.serviceRevision != service->revision)
            {
                return;
            }
            const bool evenlySpacedSlots = id == Widx::kTimetableAddSlot && inputTarget.evenlySpacedSlots;
            const bool optionalValue = id == Widx::kTimetableTravel || id == Widx::kTimetableDwell;
            uint32_t value{};
            const std::string_view text(input);
            if (text.empty())
            {
                if (!optionalValue)
                {
                    Windows::Error::open(StringIds::cant_change_timetable, StringIds::timetable_invalid_value);
                    return;
                }
                value = GameCommands::VehicleTimetableArgs::kClearValue;
            }
            else
            {
                const auto parsed = parseTimetableValue(text, id != Widx::kTimetableClockRate && !evenlySpacedSlots);
                if (!parsed.has_value() || (optionalValue && *parsed == GameCommands::VehicleTimetableArgs::kClearValue))
                {
                    Windows::Error::open(StringIds::cant_change_timetable, StringIds::timetable_invalid_value);
                    return;
                }
                value = *parsed;
            }

            using Action = GameCommands::VehicleTimetableArgs::Action;
            Action action;
            if (id == Widx::kTimetableTravel)
            {
                action = Action::setTravelMinutes;
            }
            else if (id == Widx::kTimetableDwell)
            {
                action = Action::setDwellMinutes;
            }
            else if (id == Widx::kTimetablePeriod)
            {
                action = Action::setDispatchPeriod;
            }
            else if (id == Widx::kTimetablePhase)
            {
                action = Action::setDispatchPhase;
            }
            else if (id == Widx::kTimetableMaxDelay)
            {
                action = Action::setDispatchMaxDelay;
            }
            else if (id == Widx::kTimetableAddSlot)
            {
                action = evenlySpacedSlots ? Action::setEvenlySpacedSlots : Action::addDispatchSlot;
            }
            else if (id == Widx::kTimetableClockRate)
            {
                action = Action::setClockRate;
            }
            else
            {
                return;
            }
            uint16_t orderIndex = 0;
            if (id == Widx::kTimetableClockRate)
            {
                if (value < Vehicles::TimetableManager::kMinTicksPerMinute || value > Vehicles::TimetableManager::kMaxTicksPerMinute)
                {
                    Windows::Error::open(StringIds::cant_change_timetable, StringIds::timetable_invalid_value);
                    return;
                }
                if (value != Vehicles::TimetableManager::getTicksPerMinute()
                    && !confirmTimetableAction(StringIds::timetable_clock_rate_confirm_prompt, StringIds::timetable_clock_rate_confirm_button))
                {
                    return;
                }
            }
            else
            {
                const auto* entry = findTimetableEntry(inputTarget.service, inputTarget.entry);
                if (entry == nullptr)
                {
                    Windows::Error::open(StringIds::cant_change_timetable, StringIds::timetable_invalid_value);
                    return;
                }
                orderIndex = entry->orderIndex;
            }
            timetableCommand(*head, action, orderIndex, value);
        }

        // 0x004B4B43
        static void onMouseUp(Window& self, const WidgetIndex_t widgetIndex, const WidgetId id)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            switch (id)
            {
                case Common::Widx::kCloseButton:
                    WindowManager::close(&self);
                    break;
                case Common::Widx::kCaption:
                    Common::renameVehicle(self, widgetIndex);
                    break;
                case Common::Widx::kTabMain:
                case Common::Widx::kTabDetails:
                case Common::Widx::kTabCargo:
                case Common::Widx::kTabFinances:
                case Common::Widx::kTabRoute:
                    _timetableSelections.erase(head->id);
                    _timetableDisplayTimes.erase(head->id);
                    Common::switchTab(self, widgetIndex);
                    break;
                case Widx::kOrderDelete:
                {

                    onOrderDelete(head, self.orderTableIndex);
                    if (self.orderTableIndex == -1)
                    {
                        return;
                    }

                    // Refresh selection (check if we are now at no order selected)
                    auto* order = getOrderTable(head).atIndex(self.orderTableIndex);

                    // If no order selected anymore
                    if (order == nullptr)
                    {
                        self.orderTableIndex = -1;
                    }
                    break;
                }
                case Widx::kLocalMode:
                {
                    if (!CompanyManager::isPlayerCompany(head->owner))
                    {
                        return;
                    }

                    Vehicles::Vehicle train(*head);
                    if ((train.veh1->var_48 & Vehicles::Flags48::expressMode) != Vehicles::Flags48::none)
                    {
                        GameCommands::setErrorTitle(StringIds::empty);
                        VehicleChangeRunningModeArgs args{};
                        args.head = head->id;
                        args.mode = VehicleChangeRunningModeArgs::Mode::toggleLocalExpress;
                        GameCommands::doCommand(args, GameCommands::Flags::apply);
                    }
                    break;
                }
                case Widx::kExpressMode:
                {
                    if (!CompanyManager::isPlayerCompany(head->owner))
                    {
                        return;
                    }

                    Vehicles::Vehicle train(*head);
                    if ((train.veh1->var_48 & Vehicles::Flags48::expressMode) == Vehicles::Flags48::none)
                    {
                        GameCommands::setErrorTitle(StringIds::empty);
                        VehicleChangeRunningModeArgs args{};
                        args.head = head->id;
                        args.mode = VehicleChangeRunningModeArgs::Mode::toggleLocalExpress;
                        GameCommands::doCommand(args, GameCommands::Flags::apply);
                    }
                    break;
                }
                case Widx::kOrderUnbunch:
                {
                    if (head->owner != CompanyManager::getControllingId() || self.orderTableIndex < 0)
                    {
                        return;
                    }

                    auto* order = getOrderTable(head).atIndex(self.orderTableIndex);
                    if (order == nullptr || !order->is<Vehicles::OrderStopAt>())
                    {
                        return;
                    }

                    GameCommands::setErrorTitle(StringIds::cannot_change_unbunching);
                    GameCommands::VehicleOrderToggleUnbunchingArgs args{};
                    args.head = head->id;
                    args.orderOffset = order->getOffset() - head->orderTableOffset;
                    GameCommands::doCommand(args, GameCommands::Flags::apply);
                    break;
                }
                case Widx::kOrderSkip:
                {
                    GameCommands::setErrorTitle(StringIds::empty);
                    GameCommands::VehicleOrderSkipArgs args{};
                    args.head = EntityId(self.number);
                    GameCommands::doCommand(args, GameCommands::Flags::apply);
                    break;
                }
                case Widx::kOrderUp:
                    if (onOrderMove(head, self.orderTableIndex, orderUpCommand))
                    {
                        if (self.orderTableIndex <= 0)
                        {
                            return;
                        }
                        self.orderTableIndex--;
                    }
                    break;
                case Widx::kOrderDown:
                    if (onOrderMove(head, self.orderTableIndex, orderDownCommand))
                    {
                        if (self.orderTableIndex < 0)
                        {
                            return;
                        }
                        auto* order = getOrderTable(head).atIndex(self.orderTableIndex);
                        if (order != nullptr)
                        {
                            self.orderTableIndex++;
                        }
                    }
                    break;
                case Widx::kOrderReverse:
                {
                    orderReverseCommand(head);
                    break;
                }
                case Widx::kSharedOrderLeave:
                {
                    if (head->owner != CompanyManager::getControllingId() || !Vehicles::SharedOrderManager::isShared(head->id))
                    {
                        return;
                    }

                    GameCommands::setErrorTitle(StringIds::cant_change_shared_orders);
                    GameCommands::VehicleOrderShareArgs args{};
                    args.target = head->id;
                    args.mode = GameCommands::VehicleOrderShareArgs::Mode::leave;
                    if (GameCommands::doCommand(args, GameCommands::Flags::apply) != GameCommands::kFailure)
                    {
                        _timetableSelections.erase(head->id);
                    }
                    break;
                }
                case Widx::kSharedOrderReplace:
                    if (head->owner == CompanyManager::getControllingId())
                    {
                        Vehicles::VehicleReplacement::schedule(head->id);
                    }
                    break;
                case Widx::kTimetableToggle:
                {
                    const bool enabled = Vehicles::TimetableManager::getServiceId(head->id) != Vehicles::TimetableManager::kInvalidServiceId;
                    if (enabled && !confirmTimetableAction(StringIds::timetable_delete_prompt, StringIds::timetable_delete_button))
                    {
                        return;
                    }
                    timetableCommand(
                        *head,
                        GameCommands::VehicleTimetableArgs::Action::setEnabled,
                        0,
                        !enabled);
                    break;
                }
                case Widx::kTimetableTravel:
                case Widx::kTimetableDwell:
                case Widx::kTimetablePeriod:
                case Widx::kTimetablePhase:
                case Widx::kTimetableMaxDelay:
                case Widx::kTimetableClockRate:
                    openTimetableInput(self, widgetIndex, id);
                    break;
                case Widx::kTimetableClearDispatch:
                    if (self.orderTableIndex >= 0)
                    {
                        if (!confirmTimetableAction(StringIds::timetable_clear_dispatch_prompt, StringIds::timetable_clear_dispatch))
                        {
                            return;
                        }
                        timetableCommand(
                            *head,
                            GameCommands::VehicleTimetableArgs::Action::clearDispatch,
                            self.orderTableIndex,
                            0);
                    }
                    break;
                case Widx::kTimetableResetDispatch:
                    if (!confirmTimetableAction(StringIds::timetable_restart_dispatch_prompt, StringIds::timetable_restart_dispatch_button))
                    {
                        return;
                    }
                    timetableCommand(*head, GameCommands::VehicleTimetableArgs::Action::resetDispatch, 0, 0);
                    break;
            }
        }

        static bool canUseSharedOrdersFrom(const Vehicles::VehicleHead& target, const Vehicles::VehicleHead& source)
        {
            return target.id != source.id
                && target.owner == source.owner
                && Vehicles::SharedOrderManager::areVehiclesCompatible(target, source);
        }

        static void showSharedOrderVehicleDropdownPage(Window& self, const WidgetIndex_t widgetIndex)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr || _sharedOrderDropdownVehicles.empty())
            {
                return;
            }

            const auto pageEnd = std::min(_sharedOrderDropdownPageStart + kVehicleDropdownPageSize, _sharedOrderDropdownVehicles.size());
            size_t dropdownIndex = 0;
            size_t selectedIndex = std::numeric_limits<size_t>::max();
            for (auto i = _sharedOrderDropdownPageStart; i < pageEnd; ++i)
            {
                const auto* vehicle = EntityManager::get<Vehicles::VehicleHead>(_sharedOrderDropdownVehicles[i]);
                if (vehicle != nullptr)
                {
                    FormatArguments args{};
                    args.push(vehicle->name);
                    args.push(vehicle->ordinalNumber);
                    Dropdown::add(dropdownIndex, StringIds::dropdown_stringid, args);
                    if (vehicle->id == head->id)
                    {
                        selectedIndex = dropdownIndex;
                    }
                }
                else
                {
                    Dropdown::add(dropdownIndex, StringIds::empty);
                }
                ++dropdownIndex;
            }
            if (_sharedOrderDropdownVehicles.size() > kVehicleDropdownPageSize)
            {
                Dropdown::add(dropdownIndex++, StringIds::dropdown_stringid, StringIds::more_vehicles);
            }

            const auto& widget = self.widgets[widgetIndex];
            Dropdown::showText(
                self.x + widget.left,
                self.y + widget.top,
                std::max<int16_t>(widget.width(), 160),
                widget.height(),
                self.getColour(WindowColour::secondary),
                dropdownIndex,
                0);
            if (selectedIndex != std::numeric_limits<size_t>::max())
            {
                Dropdown::setHighlightedItem(selectedIndex);
            }
        }

        static void showSharedOrderVehicleDropdown(Window& self, const WidgetIndex_t widgetIndex)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }

            const bool showMembers = Vehicles::SharedOrderManager::isShared(head->id);
            const auto mode = showMembers ? SharedOrderDropdownMode::members : SharedOrderDropdownMode::joinSource;
            const auto requestedPage = _sharedOrderDropdownTarget == head->id && _sharedOrderDropdownMode == mode
                ? _sharedOrderDropdownPageStart
                : 0;

            _sharedOrderDropdownVehicles.clear();
            _sharedOrderDropdownPageStart = 0;
            _sharedOrderDropdownTarget = EntityId::null;
            _sharedOrderDropdownMode = SharedOrderDropdownMode::none;
            if (showMembers)
            {
                _sharedOrderDropdownVehicles = Vehicles::SharedOrderManager::getMembers(head->id);
            }
            else
            {
                if (head->owner != CompanyManager::getControllingId())
                {
                    return;
                }
                for (const auto* source : VehicleManager::VehicleList())
                {
                    if (canUseSharedOrdersFrom(*head, *source))
                    {
                        _sharedOrderDropdownVehicles.push_back(source->id);
                    }
                }
            }

            std::ranges::sort(_sharedOrderDropdownVehicles, {}, [](const EntityId id) { return enumValue(id); });
            if (_sharedOrderDropdownVehicles.empty())
            {
                return;
            }

            _sharedOrderDropdownPageStart = requestedPage < _sharedOrderDropdownVehicles.size() ? requestedPage : 0;
            _sharedOrderDropdownTarget = head->id;
            _sharedOrderDropdownMode = mode;
            showSharedOrderVehicleDropdownPage(self, widgetIndex);
        }

        // 0x004B4DD3
        static void createOrderDropdown(Window& self, const WidgetIndex_t i, const StringId orderType)
        {
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            auto index = 0;
            for (uint16_t cargoId = 0; cargoId < ObjectManager::getMaxObjects(ObjectType::cargo); ++cargoId)
            {
                if (!(head->trainAcceptedCargoTypes & (1 << cargoId)))
                {
                    continue;
                }

                auto cargoObj = ObjectManager::get<CargoObject>(cargoId);
                FormatArguments args{};
                args.push(cargoObj->name);
                args.push(cargoObj->unitInlineSprite);
                args.push(cargoId);
                Dropdown::add(index, orderType, args);
                index++;
            }

            auto x = self.widgets[i].left + self.x;
            auto y = self.widgets[i].top + self.y;
            auto width = self.widgets[i].width();
            auto height = self.widgets[i].height();
            Dropdown::showText(x, y, width, height, self.getColour(WindowColour::secondary), index, 0);
            Dropdown::setHighlightedItem(0);
        }

        // 0x004B4B8C
        static void onMouseDown(Window& self, const WidgetIndex_t i, const WidgetId id)
        {
            switch (id)
            {
                case Widx::kOrderForceUnload:
                    createOrderDropdown(self, i, StringIds::orders_unload_all2);
                    break;
                case Widx::kOrderWait:
                    createOrderDropdown(self, i, StringIds::orders_wait_for_full_load_of2);
                    break;
                case Widx::kSharedOrderPrimary:
                    showSharedOrderVehicleDropdown(self, i);
                    break;
                case Widx::kTimetableSlots:
                    showTimetableSlotDropdown(self, i);
                    break;
                case Widx::kTimetableAddSlot:
                    showTimetableAddSlotDropdown(self, i);
                    break;
            }
        }

        // order : al (first 3 bits)
        // order argument : eax (3 - 32 bits), cx
        // Note will move orders so do not use while iterating OrderRingView
        // 0x004B4ECB
        static void addNewOrder(Window& self, const Vehicles::Order& order)
        {
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            auto chosenOffset = head->sizeOfOrderTable - 1;
            if (self.orderTableIndex != -1)
            {
                auto* chosenOrder = getOrderTable(head).atIndex(self.orderTableIndex);
                if (chosenOrder != nullptr)
                {
                    chosenOffset = chosenOrder->getOffset() - head->orderTableOffset;
                }
            }

            GameCommands::setErrorTitle(StringIds::orders_cant_insert);
            auto previousSize = head->sizeOfOrderTable;

            GameCommands::VehicleOrderInsertArgs args{};
            args.head = head->id;
            args.orderOffset = chosenOffset;
            args.rawOrder = order.getRaw();
            GameCommands::doCommand(args, GameCommands::Flags::apply);

            Vehicles::OrderManager::generateNumDisplayFrames(head);

            if (head->sizeOfOrderTable == previousSize)
            {
                return;
            }

            if (self.orderTableIndex == -1)
            {
                return;
            }

            self.orderTableIndex++;
        }

        static void copyOrdersFrom(Window& self, const Vehicles::VehicleHead& source)
        {
            Audio::playSound(Audio::SoundId::waypoint, Audio::ChannelId::ui, Input::getDragLastLocation().x);
            std::vector<std::shared_ptr<Vehicles::Order>> clonedOrders;
            for (auto& order : getOrderTable(&source))
            {
                clonedOrders.push_back(order.clone());
            }
            for (auto& order : clonedOrders)
            {
                addNewOrder(self, *order);
            }
            WindowManager::bringToFront(self);
        }

        static void joinSharedOrdersFrom(Window& self, const Vehicles::VehicleHead& source)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr
                || head->owner != CompanyManager::getControllingId()
                || Vehicles::SharedOrderManager::isShared(head->id)
                || !canUseSharedOrdersFrom(*head, source))
            {
                return;
            }
            const auto targetService = Vehicles::TimetableManager::getServiceId(head->id);
            const auto sourceService = Vehicles::TimetableManager::getServiceId(source.id);
            const bool replacesOrders = head->sizeOfOrderTable > sizeof(Vehicles::OrderEnd)
                && !Vehicles::SharedOrderManager::areOrdersEqual(*head, source);
            const bool replacesTimetable = targetService != Vehicles::TimetableManager::kInvalidServiceId && targetService != sourceService;
            if (replacesOrders || replacesTimetable)
            {
                FormatArguments promptArgs{};
                if (!Windows::PromptOkCancel::open(
                        StringIds::cant_change_shared_orders,
                        replacesTimetable ? StringIds::replace_with_shared_orders_and_timetable_prompt : StringIds::replace_with_shared_orders_prompt,
                        promptArgs,
                        StringIds::share_orders))
                {
                    return;
                }
            }

            GameCommands::setErrorTitle(StringIds::cant_change_shared_orders);
            GameCommands::VehicleOrderShareArgs args{};
            args.target = head->id;
            args.source = source.id;
            args.mode = GameCommands::VehicleOrderShareArgs::Mode::joinSource;
            if (GameCommands::doCommand(args, GameCommands::Flags::apply) != GameCommands::kFailure)
            {
                self.orderTableIndex = -1;
                _timetableSelections.erase(head->id);
                self.rowHover = -1;
                Vehicles::OrderManager::generateNumDisplayFrames(head);
            }
        }

        // 0x004B4BAC
        static void onDropdown(Window& self, const WidgetIndex_t i, const WidgetId id, const int16_t dropdownIndex)
        {
            auto item = dropdownIndex == -1 ? Dropdown::getHighlightedItem() : dropdownIndex;
            if (item < 0)
            {
                return;
            }
            switch (id)
            {
                case Widx::kTimetableAddSlot:
                    if (item <= 1)
                    {
                        openTimetableInput(self, i, id, item == 1);
                    }
                    break;
                case Widx::kTimetableSlots:
                {
                    if (_timetableDropdownTarget.vehicle != EntityId(self.number)
                        || _timetableSlotPageStart >= _timetableDropdownSlots.size())
                    {
                        return;
                    }
                    const auto pageSize = std::min(kTimetableSlotPageSize, _timetableDropdownSlots.size() - _timetableSlotPageStart);
                    if (item > static_cast<int16_t>(pageSize))
                    {
                        return;
                    }
                    const auto pageEnd = _timetableSlotPageStart + pageSize;
                    const bool hasPageLink = pageEnd < _timetableDropdownSlots.size() || _timetableSlotPageStart != 0;
                    if (item == static_cast<int16_t>(pageSize))
                    {
                        if (!hasPageLink)
                        {
                            return;
                        }
                        _timetableSlotPageStart = pageEnd < _timetableDropdownSlots.size() ? pageEnd : 0;
                        showTimetableSlotDropdownPage(self, i);
                        return;
                    }

                    auto* head = Common::getVehicle(self);
                    const auto* service = head != nullptr ? Vehicles::TimetableManager::getServiceForVehicle(head->id) : nullptr;
                    auto* entry = service != nullptr && service->id == _timetableDropdownTarget.service
                            && service->revision == _timetableDropdownTarget.serviceRevision
                        ? findTimetableEntry(_timetableDropdownTarget.service, _timetableDropdownTarget.entry)
                        : nullptr;
                    const auto slot = _timetableDropdownSlots[_timetableSlotPageStart + item];
                    if (head != nullptr && entry != nullptr && entry->dispatch.has_value()
                        && std::ranges::find(entry->dispatch->slots, slot) != entry->dispatch->slots.end())
                    {
                        timetableCommand(
                            *head,
                            GameCommands::VehicleTimetableArgs::Action::removeDispatchSlot,
                            entry->orderIndex,
                            slot);
                    }
                    _timetableDropdownSlots.clear();
                    _timetableDropdownSlotLabels.clear();
                    _timetableDropdownTarget = {};
                    break;
                }
                case Widx::kSharedOrderPrimary:
                {
                    if (_sharedOrderDropdownTarget != EntityId(self.number)
                        || _sharedOrderDropdownPageStart >= _sharedOrderDropdownVehicles.size())
                    {
                        return;
                    }

                    const auto pageSize = std::min(kVehicleDropdownPageSize, _sharedOrderDropdownVehicles.size() - _sharedOrderDropdownPageStart);
                    if (item > static_cast<int16_t>(pageSize))
                    {
                        return;
                    }
                    if (item == static_cast<int16_t>(pageSize))
                    {
                        if (_sharedOrderDropdownVehicles.size() <= kVehicleDropdownPageSize)
                        {
                            return;
                        }
                        const auto nextPage = _sharedOrderDropdownPageStart + pageSize;
                        _sharedOrderDropdownPageStart = nextPage < _sharedOrderDropdownVehicles.size() ? nextPage : 0;
                        showSharedOrderVehicleDropdown(self, i);
                        return;
                    }

                    const auto selectedId = _sharedOrderDropdownVehicles[_sharedOrderDropdownPageStart + item];
                    const auto mode = _sharedOrderDropdownMode;
                    _sharedOrderDropdownMode = SharedOrderDropdownMode::none;
                    _sharedOrderDropdownTarget = EntityId::null;
                    _sharedOrderDropdownVehicles.clear();

                    auto* head = Common::getVehicle(self);
                    auto* selected = EntityManager::get<Vehicles::VehicleHead>(selectedId);
                    if (head == nullptr || selected == nullptr)
                    {
                        return;
                    }

                    if (mode == SharedOrderDropdownMode::members)
                    {
                        if (Vehicles::SharedOrderManager::getGroupId(head->id) == EntityId::null
                            || Vehicles::SharedOrderManager::getGroupId(head->id) != Vehicles::SharedOrderManager::getGroupId(selected->id))
                        {
                            return;
                        }
                        auto* memberWindow = Main::open(selected);
                        if (memberWindow != nullptr)
                        {
                            memberWindow->callOnMouseUp(Common::widx::tabRoute, memberWindow->widgets[Common::widx::tabRoute].id);
                        }
                        return;
                    }

                    if (mode != SharedOrderDropdownMode::joinSource)
                    {
                        return;
                    }
                    joinSharedOrdersFrom(self, *selected);
                    break;
                }
                case Widx::kOrderForceUnload:
                {
                    Vehicles::OrderUnloadAll unload(Dropdown::getItemArgument(item, 3));
                    addNewOrder(self, unload);
                    break;
                }
                case Widx::kOrderWait:
                {
                    Vehicles::OrderWaitFor wait(Dropdown::getItemArgument(item, 3));
                    addNewOrder(self, wait);
                    break;
                }
            }
        }

        // 0x004B55D1
        // "Show <vehicle> route details" tab in vehicle window
        static void onUpdate(Window& self)
        {
            self.frameNo += 1;
            self.callPrepareDraw();

            WindowManager::invalidateWidget(WindowType::vehicle, self.number, 8);
            self.invalidate();

            if (!WindowManager::isInFront(&self))
            {
                return;
            }
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            if (head->owner != CompanyManager::getControllingId())
            {
                return;
            }

            if (!ToolManager::isToolActive(WindowType::vehicle, self.number))
            {
                if (ToolManager::toolSet(self, widx::tool, CursorId::crosshair))
                {
                    self.invalidate();
                    Vehicles::OrderManager::generateNumDisplayFrames(head);
                }
            }
        }

        // 0x004B4D74
        static std::optional<FormatArguments> tooltip(Ui::Window& self, WidgetIndex_t, [[maybe_unused]] const WidgetId id)
        {
            FormatArguments args{};
            args.push(StringIds::tooltip_scroll_orders_list);
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return {};
            }
            args.push(StringIds::getVehicleType(head->vehicleType));
            return args;
        }

        // 0x004B5BB9
        static ViewportInteraction::InteractionArg stationLabelAdjustedInteraction(const Vehicles::VehicleHead& head, int16_t orderTableIndex, StationId stationId, ViewportInteraction::InteractionArg interaction)
        {
            auto* station = StationManager::get(stationId);
            if (station == nullptr)
            {
                return ViewportInteraction::kNoInteractionArg;
            }
            if (station->owner != CompanyManager::getControllingId())
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            Input::setHoveredStationId(stationId);
            World::setMapSelectionFlags(World::MapSelectionFlags::hoveringOverStation);
            ViewportManager::invalidate(station);

            {
                auto args = FormatArguments::mapToolTip(StringIds::click_to_insert_new_order_stop_at);
                args.push(station->name);
                args.push(station->town);
            }

            if (orderTableIndex != 0)
            {
                uint32_t targetOffset = 0U;
                Vehicles::OrderRingView orders(head.orderTableOffset);
                auto lastOrder = orders.begin();
                if (lastOrder != orders.end())
                {
                    if (orderTableIndex < 0)
                    {
                        while ((lastOrder + 1) != orders.end())
                        {
                            lastOrder++;
                        }
                    }
                    else
                    {
                        while ((lastOrder + 1) != orders.end() && orderTableIndex != 0)
                        {
                            lastOrder++;
                            orderTableIndex--;
                        }
                    }
                }
                targetOffset = lastOrder->getOffset();

                auto* order = Vehicles::OrderManager::orders()[targetOffset].as<Vehicles::OrderStopAt>();
                if (order != nullptr)
                {
                    if (order->getStation() == stationId)
                    {
                        if (head.mode != TransportMode::water && head.mode != TransportMode::air)
                        {
                            auto args = FormatArguments::mapToolTip(StringIds::click_again_to_change_last_order_route_through);
                            args.push(station->name);
                            args.push(station->town);
                        }
                    }
                }
            }
            return ViewportInteraction::InteractionArg(interaction.pos, enumValue(stationId), ViewportInteraction::InteractionItem::stationLabel, interaction.modId);
        }

        // 0x004B5BA3
        static ViewportInteraction::InteractionArg stationAdjustedInteraction(const Vehicles::VehicleHead& head, int16_t orderTableIndex, World::StationElement* elStation, ViewportInteraction::InteractionArg interaction)
        {
            if (elStation == nullptr)
            {
                return ViewportInteraction::kNoInteractionArg;
            }
            if (elStation->isAiAllocated() || elStation->isGhost())
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            return stationLabelAdjustedInteraction(head, orderTableIndex, elStation->stationId(), interaction);
        }

        // 0x004B5B92
        static ViewportInteraction::InteractionArg trainStationAdjustedInteraction(const Vehicles::VehicleHead& head, int16_t orderTableIndex, ViewportInteraction::InteractionArg interaction)
        {
            auto* entry = static_cast<World::TileElementEntry*>(interaction.object);
            auto* elStation = entry->as<StationElement>();
            if (elStation == nullptr)
            {
                return ViewportInteraction::kNoInteractionArg;
            }
            auto* elTrack = entry->prev()->as<TrackElement>();
            if (elTrack == nullptr)
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            if (elTrack->owner() != CompanyManager::getControllingId())
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            return stationAdjustedInteraction(head, orderTableIndex, elStation, interaction);
        }

        // 0x004B5AC9
        static ViewportInteraction::InteractionArg trackAdjustedInteraction(const Vehicles::VehicleHead& head, int16_t orderTableIndex, ViewportInteraction::InteractionArg interaction)
        {
            auto* entry = static_cast<World::TileElementEntry*>(interaction.object);
            auto* elTrack = entry->as<TrackElement>();
            if (elTrack == nullptr)
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            if (elTrack->isAiAllocated() || elTrack->isGhost())
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            if (elTrack->owner() != CompanyManager::getControllingId())
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            if (elTrack->hasStationElement())
            {
                auto* stationEntry = entry->next();
                auto* elStation = stationEntry->as<StationElement>();
                if (elStation != nullptr)
                {
                    if (!elStation->isAiAllocated() && !elStation->isGhost())
                    {
                        ViewportInteraction::InteractionArg arg{ interaction.pos, 0, interaction.type, interaction.modId };
                        arg.object = stationEntry;
                        return trainStationAdjustedInteraction(head, orderTableIndex, arg);
                    }
                }
            }

            if (head.mode == TransportMode::water || head.mode == TransportMode::air)
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            FormatArguments::mapToolTip(StringIds::click_to_insert_new_order_route_through);
            return interaction;
        }

        // 0x004B5AC9
        static ViewportInteraction::InteractionArg roadAdjustedInteraction(const Vehicles::VehicleHead& head, int16_t orderTableIndex, ViewportInteraction::InteractionArg interaction)
        {
            auto* entry = static_cast<World::TileElementEntry*>(interaction.object);
            auto* elRoad = entry->as<RoadElement>();
            if (elRoad == nullptr)
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            if (elRoad->isAiAllocated() || elRoad->isGhost())
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            if (elRoad->hasStationElement())
            {
                auto* elStation = getStationElement({ interaction.pos, elRoad->baseHeight() });
                if (elStation != nullptr && !elStation->isAiAllocated() && !elStation->isGhost())
                {
                    return stationAdjustedInteraction(head, orderTableIndex, elStation, interaction);
                }
            }
            if (head.mode == TransportMode::air)
            {
                return ViewportInteraction::kNoInteractionArg;
            }
            FormatArguments::mapToolTip(StringIds::click_to_insert_new_order_route_through);
            return interaction;
        }

        // 0x004B5B7F
        static ViewportInteraction::InteractionArg dockAirportAdjustedInteraction(const Vehicles::VehicleHead& head, int16_t orderTableIndex, ViewportInteraction::InteractionArg interaction)
        {
            auto* entry = static_cast<World::TileElementEntry*>(interaction.object);
            auto* elStation = entry->as<StationElement>();
            if (elStation == nullptr)
            {
                return ViewportInteraction::kNoInteractionArg;
            }
            if (elStation->owner() != CompanyManager::getControllingId())
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            return stationAdjustedInteraction(head, orderTableIndex, elStation, interaction);
        }

        // 0x004B5A9B
        static ViewportInteraction::InteractionArg waterAdjustedInteraction(const Vehicles::VehicleHead& head, ViewportInteraction::InteractionArg interaction)
        {
            if (head.mode != TransportMode::water)
            {
                return ViewportInteraction::kNoInteractionArg;
            }
            FormatArguments::mapToolTip(StringIds::click_to_insert_new_order_route_through);
            return interaction;
        }

        // 0x004B5A1A
        static Ui::ViewportInteraction::InteractionArg getRouteInteractionFromCursor(Window& self, const int16_t x, const int16_t y)
        {
            auto* head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return ViewportInteraction::kNoInteractionArg;
            }

            auto flags = ViewportInteraction::InteractionItemFlags::track
                | ViewportInteraction::InteractionItemFlags::roadAndTram
                | ViewportInteraction::InteractionItemFlags::station
                | ViewportInteraction::InteractionItemFlags::stationLabel;

            if (head->mode == TransportMode::water)
            {
                flags = ViewportInteraction::InteractionItemFlags::water
                    | ViewportInteraction::InteractionItemFlags::station
                    | ViewportInteraction::InteractionItemFlags::stationLabel;
            }
            auto [interaction, viewport] = ViewportInteraction::getMapCoordinatesFromPos(x, y, ~flags);

            switch (interaction.type)
            {
                case ViewportInteraction::InteractionItem::track:
                    return trackAdjustedInteraction(*head, self.orderTableIndex, interaction);

                case ViewportInteraction::InteractionItem::road:
                    return roadAdjustedInteraction(*head, self.orderTableIndex, interaction);

                case ViewportInteraction::InteractionItem::trainStation:
                    return trainStationAdjustedInteraction(*head, self.orderTableIndex, interaction);

                case ViewportInteraction::InteractionItem::roadStation:
                {
                    auto* entry = static_cast<World::TileElementEntry*>(interaction.object);
                    return stationAdjustedInteraction(*head, self.orderTableIndex, entry->as<World::StationElement>(), interaction);
                }

                case ViewportInteraction::InteractionItem::airport:
                case ViewportInteraction::InteractionItem::dock:
                    return dockAirportAdjustedInteraction(*head, self.orderTableIndex, interaction);

                case ViewportInteraction::InteractionItem::stationLabel:
                    return stationLabelAdjustedInteraction(*head, self.orderTableIndex, static_cast<StationId>(interaction.value), interaction);

                case ViewportInteraction::InteractionItem::water:
                    return waterAdjustedInteraction(*head, interaction);

                default:
                    return ViewportInteraction::kNoInteractionArg;
            }
        }

        // 0x004B5088
        static void toolCancel(Window& self, [[maybe_unused]] const WidgetIndex_t widgetIdx, [[maybe_unused]] const WidgetId id)
        {
            self.invalidate();
            World::resetMapSelectionFlag(World::MapSelectionFlags::unk_04);
            Gfx::invalidateScreen();
        }

        static Vehicles::VehicleHead* getVehicleFromCursor(const int16_t x, const int16_t y)
        {
            const auto interaction = ViewportInteraction::getItemLeft(x, y);
            if (interaction.type != ViewportInteraction::InteractionItem::entity)
            {
                return nullptr;
            }

            auto* entity = reinterpret_cast<EntityBase*>(interaction.object);
            auto* vehicle = entity->asBase<Vehicles::VehicleBase>();
            return vehicle == nullptr ? nullptr : EntityManager::get<Vehicles::VehicleHead>(vehicle->getHead());
        }

        static void onToolDown(Window& self, [[maybe_unused]] const WidgetIndex_t widgetIndex, [[maybe_unused]] const WidgetId id, const int16_t x, const int16_t y)
        {
            if (auto* source = getVehicleFromCursor(x, y); source != nullptr && source->id != EntityId(self.number))
            {
                if (Input::hasKeyModifier(Input::KeyModifier::control))
                {
                    joinSharedOrdersFrom(self, *source);
                }
                else
                {
                    copyOrdersFrom(self, *source);
                }
                return;
            }

            const auto args = getRouteInteractionFromCursor(self, x, y);
            switch (args.type)
            {
                case Ui::ViewportInteraction::InteractionItem::track:
                {
                    // 0x004B5160
                    auto entry = static_cast<World::TileElementEntry*>(args.object);
                    auto trackElement = entry->as<TrackElement>();
                    if (trackElement == nullptr)
                    {
                        break;
                    }
                    auto height = trackElement->baseHeight();
                    auto trackId = trackElement->trackId();
                    const auto& trackPiece = World::TrackData::getTrackPiece(trackId);
                    const auto& trackPart = trackPiece[trackElement->sequenceIndex()];

                    auto offsetToFirstTile = Math::Vector::rotate(Pos2{ trackPart.x, trackPart.y }, trackElement->rotation());
                    auto firstTilePos = args.pos - offsetToFirstTile;
                    const auto tPos = World::toTileSpace(firstTilePos);
                    height -= trackPart.z;

                    Vehicles::OrderRouteWaypoint waypoint(tPos, height / 8, trackElement->rotation(), trackId);
                    Audio::playSound(Audio::SoundId::waypoint, Audio::ChannelId::ui, Input::getDragLastLocation().x);
                    addNewOrder(self, waypoint);
                    break;
                }
                case Ui::ViewportInteraction::InteractionItem::water:
                {
                    // Water
                    auto heights = TileManager::getHeight(args.pos);
                    auto height = heights.landHeight;
                    if (heights.waterHeight != 0)
                    {
                        height = heights.waterHeight;
                    }
                    Audio::playSound(Audio::SoundId::waypoint, Audio::ChannelId::ui, Input::getDragLastLocation().x);

                    const auto tPos = World::toTileSpace(args.pos);
                    Vehicles::OrderRouteWaypoint waypoint(tPos, height / 8, 0, 0);
                    addNewOrder(self, waypoint);
                    break;
                }
                case Ui::ViewportInteraction::InteractionItem::stationLabel:
                {
                    Audio::playSound(Audio::SoundId::waypoint, Audio::ChannelId::ui, Input::getDragLastLocation().x);
                    const auto stationId = StationId(args.value);
                    Vehicles::OrderStopAt station(stationId);
                    addNewOrder(self, station);
                    break;
                }
                case Ui::ViewportInteraction::InteractionItem::road:
                {
                    // 0x004B5223
                    auto* entry = static_cast<World::TileElementEntry*>(args.object);
                    auto* roadElement = entry->as<RoadElement>();
                    if (roadElement == nullptr)
                    {
                        break;
                    }
                    auto height = roadElement->baseHeight();
                    auto roadId = roadElement->roadId();
                    const auto& roadPiece = World::TrackData::getRoadPiece(roadId);
                    const auto& roadPart = roadPiece[roadElement->sequenceIndex()];

                    auto offsetToFirstTile = Math::Vector::rotate(Pos2{ roadPart.x, roadPart.y }, roadElement->rotation());
                    auto firstTilePos = args.pos - offsetToFirstTile;
                    const auto tPos = World::toTileSpace(firstTilePos);
                    height -= roadPart.z;

                    Vehicles::OrderRouteWaypoint waypoint(tPos, height / 8, roadElement->rotation(), roadId);
                    Audio::playSound(Audio::SoundId::waypoint, Audio::ChannelId::ui, Input::getDragLastLocation().x);
                    addNewOrder(self, waypoint);
                    break;
                }

                default:
                    break;
            }
        }

        // 0x004B50CE
        static Ui::CursorId toolCursor(Window& self, const int16_t x, const int16_t y, const Ui::CursorId fallback, bool& out)
        {
            if (auto* source = getVehicleFromCursor(x, y); source != nullptr)
            {
                auto* head = Common::getVehicle(self);
                if (head != nullptr && source->id != head->id)
                {
                    out = true;
                    if (Input::hasKeyModifier(Input::KeyModifier::control))
                    {
                        out = !Vehicles::SharedOrderManager::isShared(head->id) && canUseSharedOrdersFrom(*head, *source);
                    }
                    return out ? CursorId::inwardArrows : fallback;
                }
            }

            const auto args = getRouteInteractionFromCursor(self, x, y);
            out = args.type != Ui::ViewportInteraction::InteractionItem::noInteraction;
            if (out)
            {
                return CursorId::inwardArrows;
            }
            return fallback;
        }

        // 0x004B4D9B
        static void getScrollSize(Ui::Window& self, [[maybe_unused]] const uint32_t scrollIndex, [[maybe_unused]] int32_t& scrollWidth, int32_t& scrollHeight)
        {
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            auto table = getOrderTable(head);
            scrollHeight = lineHeight * std::distance(table.begin(), table.end());

            // Space for the 'end of orders' item
            scrollHeight += lineHeight;
        }

        static void scrollMouseDown(Window& self, [[maybe_unused]] const int16_t x, const int16_t y, [[maybe_unused]] const uint8_t scrollIndex)
        {
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            int16_t item = y / lineHeight;
            Vehicles::Order* selectedOrder = getOrderTable(head).atIndex(item);
            if (selectedOrder == nullptr)
            {
                item = -1;
            }

            auto* toolWindow = ToolManager::toolGetActiveWindow();
            // If another vehicle window is open and has focus (tool)
            if (toolWindow != nullptr && toolWindow->type == self.type && toolWindow->number != self.number)
            {
                if (item == -1)
                {
                    // Copy complete order list
                    copyOrdersFrom(*toolWindow, *head);
                }
                else
                {
                    // Copy a single entry on the order list
                    Audio::playSound(Audio::SoundId::waypoint, Audio::ChannelId::ui, Input::getDragLastLocation().x);
                    auto clonedOrder = selectedOrder->clone();
                    addNewOrder(*toolWindow, *clonedOrder);
                    WindowManager::bringToFront(*toolWindow);
                }
                return;
            }

            if (item != self.orderTableIndex)
            {
                self.orderTableIndex = item;
                const auto* entry = item >= 0 ? Vehicles::TimetableManager::getEntry(head->id, static_cast<uint8_t>(item)) : nullptr;
                if (entry != nullptr)
                {
                    _timetableSelections[head->id] = entry->id;
                }
                else
                {
                    _timetableSelections.erase(head->id);
                }
                self.invalidate();
                return;
            }

            if (selectedOrder == nullptr)
            {
                return;
            }
            switch (selectedOrder->getType())
            {
                case Vehicles::OrderType::StopAt:
                case Vehicles::OrderType::RouteThrough:
                {
                    auto* stationOrder = static_cast<Vehicles::OrderStation*>(selectedOrder);
                    auto station = StationManager::get(stationOrder->getStation());
                    auto main = WindowManager::getMainWindow();
                    if (main)
                    {
                        main->viewportCentreOnTile({ station->x, station->y, static_cast<coord_t>(station->z + 32) });
                    }

                    break;
                }
                case Vehicles::OrderType::RouteWaypoint:
                {
                    auto* routeOrder = selectedOrder->as<Vehicles::OrderRouteWaypoint>();
                    if (routeOrder != nullptr)
                    {
                        auto main = WindowManager::getMainWindow();
                        if (main)
                        {
                            auto position = routeOrder->getWaypoint();
                            position.x += 16;
                            position.y += 16;
                            position.z += 32;
                            main->viewportCentreOnTile(position);
                        }
                    }
                    break;
                }
                case Vehicles::OrderType::UnloadAll:
                case Vehicles::OrderType::WaitFor:
                case Vehicles::OrderType::End:
                    // These orders don't have a location to centre on
                    break;
            }
        }

        // 0x004B530C
        static void scrollMouseOver(Window& self, [[maybe_unused]] const int16_t x, const int16_t y, [[maybe_unused]] const uint8_t scrollIndex)
        {
            self.flags &= ~WindowFlags::notScrollView;
            auto item = y / lineHeight;
            if (self.rowHover != item)
            {
                self.rowHover = item;
                self.invalidate();
            }
        }

        // 0x004B5339
        static Ui::CursorId cursor(Window& self, [[maybe_unused]] const WidgetIndex_t widgetIdx, const WidgetId id, [[maybe_unused]] const int16_t x, [[maybe_unused]] const int16_t y, const Ui::CursorId fallback)
        {
            if (id != Widx::kRouteList)
            {
                return fallback;
            }

            if (ToolManager::isToolActive(self.type, self.number))
            {
                return CursorId::inwardArrows;
            }
            return fallback;
        }

        // 0x004B56B8 TODO Rename
        static void createViewport(Window& self)
        {
            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            Vehicles::OrderManager::generateNumDisplayFrames(head);
        }

        // 0x004B468C
        static void prepareDraw(Window& self)
        {
            Common::setActiveTabs(self);
            Common::setCaptionEnableState(self);

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            const auto timetableService = Vehicles::TimetableManager::getServiceId(head->id);
            const bool timetableEnabled = timetableService != Vehicles::TimetableManager::kInvalidServiceId;
            const auto rememberedSelection = _timetableSelections.find(head->id);
            if (timetableEnabled && rememberedSelection != _timetableSelections.end())
            {
                const auto* entry = findTimetableEntry(timetableService, rememberedSelection->second);
                if (entry != nullptr)
                {
                    self.orderTableIndex = entry->orderIndex;
                }
                else
                {
                    self.orderTableIndex = -1;
                    _timetableSelections.erase(rememberedSelection);
                }
            }
            else if (self.orderTableIndex >= 0 && getOrderTable(head).atIndex(self.orderTableIndex) == nullptr)
            {
                self.orderTableIndex = -1;
            }
            if (!timetableEnabled)
            {
                _timetableSelections.erase(head->id);
            }
            else if (_timetableSelections.find(head->id) == _timetableSelections.end())
            {
                const auto* entry = getSelectedTimetableEntry(self, *head);
                if (entry != nullptr)
                {
                    _timetableSelections[head->id] = entry->id;
                }
            }

            // Set title.
            {
                auto args = FormatArguments(self.widgets[Common::widx::caption].textArgs);
                args.push(head->name);
                args.push(head->ordinalNumber);
            }

            self.widgets[widx::routeList].tooltip = ToolManager::isToolActive(self.type, self.number) ? StringIds::tooltip_route_scrollview_copy : StringIds::tooltip_route_scrollview;

            self.widgets[Common::widx::frame].right = self.width - 1;
            self.widgets[Common::widx::frame].bottom = self.height - 1;
            self.widgets[Common::widx::panel].right = self.width - 1;
            self.widgets[Common::widx::panel].bottom = self.height - 1;
            self.widgets[Common::widx::caption].right = self.width - 2;
            self.widgets[Common::widx::closeButton].left = self.width - 15;
            self.widgets[Common::widx::closeButton].right = self.width - 3;

            self.widgets[widx::routeList].top = 108;
            self.widgets[widx::routeList].right = self.width - 26;

            self.widgets[widx::orderForceUnload].right = self.width - 2;
            self.widgets[widx::orderWait].right = self.width - 2;
            self.widgets[widx::orderSkip].right = self.width - 2;
            self.widgets[widx::orderDelete].right = self.width - 2;
            self.widgets[widx::orderUp].right = self.width - 2;
            self.widgets[widx::orderDown].right = self.width - 2;
            self.widgets[widx::orderReverse].right = self.width - 2;

            self.widgets[widx::orderForceUnload].left = self.width - 25;
            self.widgets[widx::orderWait].left = self.width - 25;
            self.widgets[widx::orderSkip].left = self.width - 25;
            self.widgets[widx::orderDelete].left = self.width - 25;
            self.widgets[widx::orderUp].left = self.width - 25;
            self.widgets[widx::orderDown].left = self.width - 25;
            self.widgets[widx::orderReverse].left = self.width - 25;

            self.disabledWidgets |= (1 << widx::orderForceUnload) | (1 << widx::orderWait) | (1 << widx::orderSkip) | (1 << widx::orderDelete);
            if (head->sizeOfOrderTable != 1)
            {
                self.disabledWidgets &= ~((1 << widx::orderSkip) | (1 << widx::orderDelete));
            }
            if (head->trainAcceptedCargoTypes != 0)
            {
                self.disabledWidgets &= ~((1 << widx::orderWait) | (1 << widx::orderForceUnload));
            }

            self.disabledWidgets |= 1 << widx::orderUnbunch;
            self.activatedWidgets &= ~(1 << widx::orderUnbunch);
            if (self.orderTableIndex >= 0)
            {
                const auto* selectedOrder = getOrderTable(head).atIndex(self.orderTableIndex);
                const auto* stopOrder = selectedOrder != nullptr ? selectedOrder->as<Vehicles::OrderStopAt>() : nullptr;
                if (stopOrder != nullptr)
                {
                    if (head->owner == CompanyManager::getControllingId() && !timetableEnabled)
                    {
                        self.disabledWidgets &= ~(1 << widx::orderUnbunch);
                    }
                    if (stopOrder->isUnbunching())
                    {
                        self.activatedWidgets |= 1 << widx::orderUnbunch;
                    }
                }
            }

            // Express / local
            self.activatedWidgets &= ~((1 << widx::expressMode) | (1 << widx::localMode));
            Vehicles::Vehicle train(*head);
            if ((train.veh1->var_48 & Vehicles::Flags48::expressMode) != Vehicles::Flags48::none)
            {
                self.activatedWidgets |= (1 << widx::expressMode);
            }
            else
            {
                self.activatedWidgets |= (1 << widx::localMode);
            }

            const bool isOtherCompany = head->owner != CompanyManager::getControllingId();
            self.widgets[widx::orderForceUnload].hidden = isOtherCompany;
            self.widgets[widx::orderWait].hidden = isOtherCompany;
            self.widgets[widx::orderSkip].hidden = isOtherCompany;
            self.widgets[widx::orderDelete].hidden = isOtherCompany;
            self.widgets[widx::orderUp].hidden = isOtherCompany;
            self.widgets[widx::orderDown].hidden = isOtherCompany;
            self.widgets[widx::orderReverse].hidden = isOtherCompany;

            if (isOtherCompany)
            {
                self.widgets[widx::routeList].right += 22;
                self.disabledWidgets |= (1 << widx::expressMode | 1 << widx::localMode);
            }
            else
            {
                self.disabledWidgets &= ~(1 << widx::expressMode | 1 << widx::localMode);
            }

            const bool hasSharedOrders = Vehicles::SharedOrderManager::isShared(head->id);
            auto& sharedStatus = self.widgets[widx::sharedOrderStatus];
            auto& sharedPrimary = self.widgets[widx::sharedOrderPrimary];
            auto& sharedLeave = self.widgets[widx::sharedOrderLeave];
            auto& sharedReplace = self.widgets[widx::sharedOrderReplace];
            const auto contentRight = self.widgets[widx::routeList].right;
            self.widgets[widx::timetableToggle].right = contentRight;

            self.disabledWidgets &= ~((1 << widx::sharedOrderPrimary) | (1 << widx::sharedOrderLeave));
            sharedPrimary.hidden = !hasSharedOrders && isOtherCompany;
            sharedLeave.hidden = !hasSharedOrders || isOtherCompany;
            sharedReplace.hidden = !hasSharedOrders || isOtherCompany;
            if (hasSharedOrders)
            {
                sharedStatus.text = StringIds::shared_orders_vehicle_count;
                auto args = FormatArguments(sharedStatus.textArgs);
                args.push<uint16_t>(Vehicles::SharedOrderManager::getMemberCount(head->id));

                sharedPrimary.text = StringIds::shared_order_members;
                sharedPrimary.tooltip = StringIds::tooltip_shared_order_members;
                sharedPrimary.right = contentRight - (isOtherCompany ? 0 : 44);
                sharedPrimary.left = sharedPrimary.right - 63;
                sharedLeave.right = contentRight;
                sharedLeave.left = sharedLeave.right - 41;
                sharedStatus.right = sharedPrimary.left - 2;
                if (isOtherCompany)
                {
                    self.disabledWidgets |= 1 << widx::sharedOrderLeave;
                }
            }
            else
            {
                sharedStatus.text = StringIds::independent_orders;
                sharedStatus.right = std::min<int16_t>(102, contentRight);
                sharedPrimary.text = StringIds::use_shared_orders_from;
                sharedPrimary.tooltip = StringIds::tooltip_use_shared_orders_from;
                sharedPrimary.left = sharedStatus.right + 2;
                sharedPrimary.right = contentRight;
                if (isOtherCompany)
                {
                    self.disabledWidgets |= 1 << widx::sharedOrderPrimary;
                }
            }

            self.widgets[widx::expressMode].right = self.widgets[widx::routeList].right;
            self.widgets[widx::expressMode].left = (self.widgets[widx::expressMode].right - 3) / 2 + 3;
            self.widgets[widx::localMode].right = self.widgets[widx::expressMode].left - 1;
            self.widgets[widx::orderUnbunch].right = self.widgets[widx::routeList].right;

            self.disabledWidgets |= (1 << widx::orderUp) | (1 << widx::orderDown);
            if (self.orderTableIndex != -1)
            {
                self.disabledWidgets &= ~((1 << widx::orderUp) | (1 << widx::orderDown));
            }

            const auto contentLeft = self.widgets[widx::routeList].left;
            const auto editorRight = self.widgets[widx::routeList].right;
            const auto contentWidth = editorRight - contentLeft + 1;
            const auto midpoint = contentLeft + contentWidth / 2;
            const auto third = contentWidth / 3;
            const bool compactEditor = contentWidth < 360;
            const auto editorTop = self.height - (compactEditor ? 108 : 95);
            self.widgets[widx::routeList].bottom = editorTop - 4;
            const auto setPair = [&](const widx first, const widx second, const int16_t top) {
                self.widgets[first].top = top;
                self.widgets[first].bottom = top + 11;
                self.widgets[first].left = contentLeft;
                self.widgets[first].right = midpoint - 1;
                self.widgets[second].top = top;
                self.widgets[second].bottom = top + 11;
                self.widgets[second].left = midpoint;
                self.widgets[second].right = editorRight;
            };
            const auto setRow = [&](const widx first, const widx last, const int16_t top) {
                self.widgets[first].top = top;
                self.widgets[first].bottom = top + 11;
                self.widgets[first].left = contentLeft;
                self.widgets[first].right = contentLeft + third - 1;
                self.widgets[first + 1].top = top;
                self.widgets[first + 1].bottom = top + 11;
                self.widgets[first + 1].left = contentLeft + third;
                self.widgets[first + 1].right = contentLeft + third * 2 - 1;
                self.widgets[last].top = top;
                self.widgets[last].bottom = top + 11;
                self.widgets[last].left = contentLeft + third * 2;
                self.widgets[last].right = editorRight;
            };

            setPair(widx::timetableTravel, widx::timetableDwell, editorTop);
            if (compactEditor)
            {
                setPair(widx::timetablePeriod, widx::timetablePhase, editorTop + 13);
                setPair(widx::timetableMaxDelay, widx::timetableSlots, editorTop + 26);
                setPair(widx::timetableAddSlot, widx::timetableClockRate, editorTop + 39);
                setPair(widx::timetableClearDispatch, widx::timetableResetDispatch, editorTop + 52);
            }
            else
            {
                setRow(widx::timetablePeriod, widx::timetableMaxDelay, editorTop + 13);
                self.widgets[widx::timetableSlots].top = editorTop + 26;
                self.widgets[widx::timetableSlots].bottom = editorTop + 37;
                self.widgets[widx::timetableSlots].left = contentLeft;
                self.widgets[widx::timetableSlots].right = contentLeft + third * 2 - 1;
                self.widgets[widx::timetableAddSlot].top = editorTop + 26;
                self.widgets[widx::timetableAddSlot].bottom = editorTop + 37;
                self.widgets[widx::timetableAddSlot].left = contentLeft + third * 2;
                self.widgets[widx::timetableAddSlot].right = editorRight;
                setRow(widx::timetableClockRate, widx::timetableResetDispatch, editorTop + 39);
            }

            auto& displayTimes = _timetableDisplayTimes[head->id];
            auto setValueWidget = [&](const widx index, const size_t displayIndex, const StringId label, const std::optional<uint32_t> value) {
                auto& widget = self.widgets[index];
                widget.text = value.has_value() ? StringIds::timetable_value_minutes : StringIds::timetable_value_unset;
                auto args = FormatArguments(widget.textArgs);
                args.push(label);
                if (value.has_value())
                {
                    displayTimes[displayIndex] = formatTimetableDuration(*value);
                    args.push(displayTimes[displayIndex].c_str());
                }
            };

            const auto* timetableEntry = getSelectedTimetableEntry(self, *head);
            const auto* dispatch = timetableEntry != nullptr && timetableEntry->dispatch.has_value() ? &*timetableEntry->dispatch : nullptr;
            setValueWidget(widx::timetableTravel, 0, StringIds::timetable_travel, timetableEntry != nullptr ? timetableEntry->travelMinutes : std::nullopt);
            setValueWidget(widx::timetableDwell, 1, StringIds::timetable_dwell, timetableEntry != nullptr ? timetableEntry->dwellMinutes : std::nullopt);
            setValueWidget(widx::timetablePeriod, 2, StringIds::timetable_period, dispatch != nullptr ? std::optional<uint32_t>{ dispatch->periodMinutes } : std::nullopt);
            setValueWidget(widx::timetablePhase, 3, StringIds::timetable_phase, dispatch != nullptr ? std::optional<uint32_t>{ dispatch->phaseMinutes } : std::nullopt);
            setValueWidget(widx::timetableMaxDelay, 4, StringIds::timetable_max_delay, dispatch != nullptr ? std::optional<uint32_t>{ dispatch->maxDelayMinutes } : std::nullopt);
            auto& clockRate = self.widgets[widx::timetableClockRate];
            clockRate.text = StringIds::timetable_value_ticks_per_minute;
            auto clockRateArgs = FormatArguments(clockRate.textArgs);
            clockRateArgs.push(StringIds::timetable_clock_rate);
            clockRateArgs.push<int32_t>(Vehicles::TimetableManager::getTicksPerMinute());

            auto& timetableToggle = self.widgets[widx::timetableToggle];
            timetableToggle.text = timetableEnabled ? StringIds::timetable_enabled : StringIds::timetable_disabled;
            timetableToggle.tooltip = timetableEnabled ? StringIds::tooltip_timetable_disable : StringIds::tooltip_timetable_toggle;
            if (timetableEnabled)
            {
                auto timetableToggleArgs = FormatArguments(timetableToggle.textArgs);
                displayTimes[5] = formatTimetableClockTime(Vehicles::TimetableManager::getClockMinute());
                timetableToggleArgs.push(displayTimes[5].c_str());
            }
            self.widgets[widx::timetableSlots].text = StringIds::timetable_slots_count;
            auto slotArgs = FormatArguments(self.widgets[widx::timetableSlots].textArgs);
            slotArgs.push<int32_t>(dispatch != nullptr ? dispatch->slots.size() : 0);

            constexpr auto kTimetableEditorWidgets = (1ULL << widx::timetableTravel)
                | (1ULL << widx::timetableDwell)
                | (1ULL << widx::timetablePeriod)
                | (1ULL << widx::timetablePhase)
                | (1ULL << widx::timetableMaxDelay)
                | (1ULL << widx::timetableSlots)
                | (1ULL << widx::timetableAddSlot)
                | (1ULL << widx::timetableClockRate)
                | (1ULL << widx::timetableClearDispatch)
                | (1ULL << widx::timetableResetDispatch);
            self.disabledWidgets |= kTimetableEditorWidgets | (1ULL << widx::timetableToggle);
            self.activatedWidgets &= ~(1ULL << widx::timetableToggle);
            if (timetableEnabled)
            {
                self.activatedWidgets |= 1ULL << widx::timetableToggle;
            }
            if (!isOtherCompany && (!head->hasUnbunchingOrder() || timetableEnabled))
            {
                self.disabledWidgets &= ~(1ULL << widx::timetableToggle);
            }
            if (!isOtherCompany && timetableEnabled)
            {
                self.disabledWidgets &= ~((1ULL << widx::timetableClockRate) | (1ULL << widx::timetableResetDispatch));
                if (timetableEntry != nullptr)
                {
                    self.disabledWidgets &= ~(1ULL << widx::timetableTravel);
                    if (timetableEntry->orderType == Vehicles::OrderType::StopAt)
                    {
                        self.disabledWidgets &= ~((1ULL << widx::timetableDwell)
                            | (1ULL << widx::timetablePeriod)
                            | (1ULL << widx::timetablePhase)
                            | (1ULL << widx::timetableMaxDelay)
                            | (1ULL << widx::timetableAddSlot));
                        if (dispatch != nullptr)
                        {
                            self.disabledWidgets &= ~(1ULL << widx::timetableClearDispatch);
                            if (!dispatch->slots.empty())
                            {
                                self.disabledWidgets &= ~(1ULL << widx::timetableSlots);
                            }
                        }
                    }
                }
            }
            Widget::leftAlignTabs(self, Common::widx::tabMain, Common::widx::tabRoute);
        }

        // 0x004B4866
        static void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            self.draw(drawingCtx);
            Common::drawTabs(self, drawingCtx);

            if (Vehicles::TimetableManager::getServiceId(EntityId(self.number)) != Vehicles::TimetableManager::kInvalidServiceId)
            {
                const auto clockMinute = Vehicles::TimetableManager::getClockMinute();
                const auto clockTime = formatTimetableClockTime(clockMinute);
                std::string secondaryTime;
                auto statusString = StringIds::timetable_status_not_started;
                FormatArguments statusArgs{};
                statusArgs.push(clockTime.c_str());
                const auto* runtime = Vehicles::TimetableManager::getVehicleRuntime(EntityId(self.number));
                if (runtime != nullptr && runtime->timetableStarted)
                {
                    const auto rate = Vehicles::TimetableManager::getTicksPerMinute();
                    auto lateness = runtime->latenessTicks;
                    if (runtime->atTimedStop)
                    {
                        const auto now = Vehicles::TimetableManager::getClockTicks();
                        lateness = now >= runtime->scheduledDepartureTick
                            ? static_cast<int64_t>(std::min<uint64_t>(now - runtime->scheduledDepartureTick, std::numeric_limits<int64_t>::max()))
                            : -static_cast<int64_t>(std::min<uint64_t>(runtime->scheduledDepartureTick - now, std::numeric_limits<int64_t>::max()));
                    }
                    if (lateness == 0)
                    {
                        statusString = StringIds::timetable_status_on_time;
                    }
                    else
                    {
                        const auto absoluteLateness = lateness < 0
                            ? static_cast<uint64_t>(-(lateness + 1)) + 1
                            : static_cast<uint64_t>(lateness);
                        const auto roundedMinutes = absoluteLateness / rate + (absoluteLateness % rate != 0);
                        secondaryTime = formatTimetableDuration(roundedMinutes);
                        statusString = lateness < 0 ? StringIds::timetable_status_early : StringIds::timetable_status_late;
                        statusArgs.push(secondaryTime.c_str());
                    }
                    if (runtime->atTimedStop && runtime->assignedSlotMinute.has_value())
                    {
                        const auto departureMinute = runtime->scheduledDepartureTick / rate + (runtime->scheduledDepartureTick % rate != 0);
                        secondaryTime = formatTimetableClockTime(departureMinute);
                        statusString = StringIds::timetable_status_departure;
                        statusArgs = {};
                        statusArgs.push(clockTime.c_str());
                        statusArgs.push(secondaryTime.c_str());
                    }
                }
                tr.drawStringLeftClipped({ 3, static_cast<int16_t>(self.height - 28) }, self.widgets[widx::routeList].width(), Colour::black, statusString, statusArgs);

                const auto* head = Common::getVehicle(self);
                std::optional<Vehicles::TimetableManager::FleetEstimate> estimate;
                if (head != nullptr)
                {
                    if (self.orderTableIndex >= 0 && self.orderTableIndex <= std::numeric_limits<uint8_t>::max())
                    {
                        estimate = Vehicles::TimetableManager::getFleetEstimate(head->id, static_cast<uint8_t>(self.orderTableIndex));
                    }
                    if (!estimate.has_value())
                    {
                        const auto* service = Vehicles::TimetableManager::getServiceForVehicle(head->id);
                        if (service != nullptr)
                        {
                            const auto entry = std::ranges::find_if(service->entries, [](const auto& item) { return item.dispatch.has_value() && !item.dispatch->slots.empty(); });
                            if (entry != service->entries.end())
                            {
                                estimate = Vehicles::TimetableManager::getFleetEstimate(head->id, entry->orderIndex);
                            }
                        }
                    }
                }
                auto estimateString = StringIds::timetable_fleet_no_slots;
                FormatArguments estimateArgs{};
                std::string cycleTime;
                if (estimate.has_value())
                {
                    estimateString = StringIds::timetable_fleet_measuring;
                    if (estimate->sampleCount == 0)
                    {
                        estimateArgs.push<int32_t>(estimate->activeVehicles);
                    }
                    else
                    {
                        estimateString = StringIds::timetable_fleet_estimate;
                        cycleTime = formatTimetableDuration(estimate->measuredCycleMinutes);
                        estimateArgs.push<int32_t>(std::min<uint32_t>(estimate->requiredVehicles, std::numeric_limits<int32_t>::max()));
                        estimateArgs.push<int32_t>(estimate->activeVehicles);
                        estimateArgs.push(cycleTime.c_str());
                    }
                }
                tr.drawStringLeftClipped({ 3, static_cast<int16_t>(self.height - 41) }, self.width - 6, Colour::black, estimateString, estimateArgs);
            }

            if (ToolManager::isToolActive(WindowType::vehicle, self.number))
            {
                // Location at bottom left edge of window
                auto loc = Point(3, self.height - 13);
                tr.drawStringLeftClipped(loc, self.width - 14, Colour::black, StringIds::route_click_on_waypoint);
            }
        }

        const std::array<StringId, 6> orderString = {
            {
                StringIds::orders_end,
                StringIds::orders_stop_at,
                StringIds::orders_route_through,
                StringIds::orders_route_thought_waypoint,
                StringIds::orders_unload_all,
                StringIds::orders_wait_for_full_load_of,
            }
        };

        static constexpr std::array<uint32_t, 63> kNumberCircle = {
            {
                ImageIds::number_circle_01,
                ImageIds::number_circle_02,
                ImageIds::number_circle_03,
                ImageIds::number_circle_04,
                ImageIds::number_circle_05,
                ImageIds::number_circle_06,
                ImageIds::number_circle_07,
                ImageIds::number_circle_08,
                ImageIds::number_circle_09,
                ImageIds::number_circle_10,
                ImageIds::number_circle_11,
                ImageIds::number_circle_12,
                ImageIds::number_circle_13,
                ImageIds::number_circle_14,
                ImageIds::number_circle_15,
                ImageIds::number_circle_16,
                ImageIds::number_circle_17,
                ImageIds::number_circle_18,
                ImageIds::number_circle_19,
                ImageIds::number_circle_20,
                ImageIds::number_circle_21,
                ImageIds::number_circle_22,
                ImageIds::number_circle_23,
                ImageIds::number_circle_24,
                ImageIds::number_circle_25,
                ImageIds::number_circle_26,
                ImageIds::number_circle_27,
                ImageIds::number_circle_28,
                ImageIds::number_circle_29,
                ImageIds::number_circle_30,
                ImageIds::number_circle_31,
                ImageIds::number_circle_32,
                ImageIds::number_circle_33,
                ImageIds::number_circle_34,
                ImageIds::number_circle_35,
                ImageIds::number_circle_36,
                ImageIds::number_circle_37,
                ImageIds::number_circle_38,
                ImageIds::number_circle_39,
                ImageIds::number_circle_40,
                ImageIds::number_circle_41,
                ImageIds::number_circle_42,
                ImageIds::number_circle_43,
                ImageIds::number_circle_44,
                ImageIds::number_circle_45,
                ImageIds::number_circle_46,
                ImageIds::number_circle_47,
                ImageIds::number_circle_48,
                ImageIds::number_circle_49,
                ImageIds::number_circle_50,
                ImageIds::number_circle_51,
                ImageIds::number_circle_52,
                ImageIds::number_circle_53,
                ImageIds::number_circle_54,
                ImageIds::number_circle_55,
                ImageIds::number_circle_56,
                ImageIds::number_circle_57,
                ImageIds::number_circle_58,
                ImageIds::number_circle_59,
                ImageIds::number_circle_60,
                ImageIds::number_circle_61,
                ImageIds::number_circle_62,
                ImageIds::number_circle_63,
            }
        };

        // 0x004B4A58 based on
        static void drawOrderLabel(Window& self, Gfx::DrawingContext& drawingCtx, const StringId strFormat, FormatArguments& args, Vehicles::Order& order, int16_t& y, int16_t& orderNumber)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            char buffer[512];
            StringManager::formatString(buffer, std::size(buffer), strFormat, args);

            tr.setCurrentFont(Gfx::Font::medium_bold);
            tr.drawString(Point(8, y - 1), Colour::black, buffer);
            auto labelWidth = tr.getStringWidth(buffer);

            if (order.hasFlags(Vehicles::OrderFlags::HasNumber))
            {
                if (ToolManager::isToolActive(self.type, self.number))
                {
                    auto imageId = kNumberCircle[orderNumber - 1];
                    drawingCtx.drawImage(ZoomLevel::full, labelWidth + 8 + 3, y, Gfx::recolour(imageId, Colour::white));
                }
                orderNumber++;
            }
        }

        // 0x004B48BA
        static void drawScroll(Window& self, Gfx::DrawingContext& drawingCtx, [[maybe_unused]] const uint32_t i)
        {
            auto tr = Gfx::TextRenderer(drawingCtx);

            drawingCtx.clearSingle(Colours::getShade(self.getColour(WindowColour::secondary).c(), 4));

            auto head = Common::getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            Vehicles::Vehicle train(*head);

            auto rowNum = 0;
            if (head->sizeOfOrderTable == 1)
            {
                auto point = Point(8, 0);
                tr.drawStringLeft(point, Colour::black, StringIds::no_route_defined);
                rowNum++; // Used to move down the text
            }

            int16_t orderNumber = 1;
            for (auto& order : getOrderTable(head))
            {
                int16_t y = rowNum * lineHeight;
                auto strFormat = StringIds::black_stringid;
                const auto* waypoint = order.as<Vehicles::OrderRouteWaypoint>();
                if (waypoint != nullptr && Vehicles::OrderManager::getRailWaypointStatus(*head, *waypoint) != Vehicles::OrderManager::RailWaypointStatus::valid)
                {
                    strFormat = StringIds::red_stringid;
                }
                if (self.orderTableIndex == rowNum)
                {
                    drawingCtx.fillRect(0, y, self.width, y + 9, PaletteIndex::black0, Gfx::RectFlags::none);
                    strFormat = StringIds::white_stringid;
                }
                if (self.rowHover == rowNum)
                {
                    strFormat = StringIds::wcolour2_stringid;
                    drawingCtx.fillRect(0, y, self.width, y + 9, enumValue(ExtColour::unk30), Gfx::RectFlags::transparent);
                }

                auto orderStringId = orderString[static_cast<uint8_t>(order.getType())];
                const auto* stopOrder = order.as<Vehicles::OrderStopAt>();
                if (stopOrder != nullptr && stopOrder->isUnbunching())
                {
                    orderStringId = StringIds::orders_stop_at_unbunching;
                }
                FormatArguments args{};
                args.push(orderStringId);
                switch (order.getType())
                {
                    case Vehicles::OrderType::End:
                    case Vehicles::OrderType::RouteWaypoint:
                        // Fall through
                        break;
                    case Vehicles::OrderType::StopAt:
                    case Vehicles::OrderType::RouteThrough:
                    {
                        auto* stationOrder = static_cast<Vehicles::OrderStation*>(&order);
                        stationOrder->setFormatArguments(args);
                        break;
                    }
                    case Vehicles::OrderType::UnloadAll:
                    case Vehicles::OrderType::WaitFor:
                    {

                        auto* cargoOrder = static_cast<Vehicles::OrderCargo*>(&order);
                        cargoOrder->setFormatArguments(args);
                        break;
                    }
                }

                drawOrderLabel(self, drawingCtx, strFormat, args, order, y, orderNumber);
                if (head->currentOrder + head->orderTableOffset == order.getOffset())
                {
                    auto point = Point(1, y - 1);
                    tr.drawStringLeft(point, Colour::black, StringIds::orders_current_order);
                }

                rowNum++;
            }

            // Output the end of orders
            Ui::Point loc = { 8, static_cast<int16_t>(rowNum * lineHeight) };
            auto strFormat = StringIds::black_stringid;
            if (self.orderTableIndex == rowNum)
            {
                drawingCtx.fillRect(0, loc.y, self.width, loc.y + lineHeight, PaletteIndex::black0, Gfx::RectFlags::none);
                strFormat = StringIds::white_stringid;
            }
            if (self.rowHover == rowNum)
            {
                strFormat = StringIds::wcolour2_stringid;
                drawingCtx.fillRect(0, loc.y, self.width, loc.y + lineHeight, enumValue(ExtColour::unk30), Gfx::RectFlags::transparent);
            }

            loc.y -= 1;
            auto args = FormatArguments::common(orderString[0]);
            tr.drawStringLeft(loc, Colour::black, strFormat, args);
        }

        static constexpr WindowEventList kEvents = {
            .onClose = close,
            .onMouseUp = onMouseUp,
            .onMouseDown = onMouseDown,
            .onDropdown = onDropdown,
            .onUpdate = onUpdate,
            .onHandleInputBegin = listWindowOnHandleInputBegin,
            .onHandleInputEnd = listWindowOnHandleInputEnd,
            .onToolDown = onToolDown,
            .onToolAbort = toolCancel,
            .toolCursor = toolCursor,
            .getScrollSize = getScrollSize,
            .scrollMouseDown = scrollMouseDown,
            .scrollMouseOver = scrollMouseOver,
            .textInput = timetableTextInput,
            .viewportRotate = createViewport,
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

    namespace Common
    {

        struct TabInformation
        {
            const widx widgetIndex;
            std::span<const Widget> widgets;
            const WindowEventList& events;
            const uint64_t holdableWidgets;
            const Size minSize;
            const Size maxSize;
        };

        // clang-format off
        static TabInformation tabInformationByTabOffset[] = {
            { widx::tabMain,     Main::widgets,     Main::getEvents(),     Main::holdableWidgets,     Main::kMinWindowSize,     Main::kMaxWindowSize     },
            { widx::tabDetails,  Details::widgets,  Details::getEvents(),  Details::holdableWidgets,  Details::kMinWindowSize,  Details::kMaxWindowSize  },
            { widx::tabCargo,    Cargo::widgets,    Cargo::getEvents(),    Cargo::holdableWidgets,    Cargo::kMinWindowSize,    Cargo::kMaxWindowSize    },
            { widx::tabFinances, Finances::widgets, Finances::getEvents(), Finances::holdableWidgets, Finances::kMinWindowSize, Finances::kMaxWindowSize },
            { widx::tabRoute,    Route::widgets,    Route::getEvents(),    Route::holdableWidgets,    Route::kMinWindowSize,    Route::kMaxWindowSize    }
        };
        // clang-format on

        static void setActiveTabs(Window& self)
        {
            self.activatedWidgets &= ~((1 << widx::tabMain) | (1 << widx::tabDetails) | (1 << widx::tabCargo) | (1 << widx::tabFinances) | (1 << widx::tabRoute));
            self.activatedWidgets |= 1ULL << (widx::tabMain + self.currentTab);
        }

        static std::pair<uint32_t, StringId> getPickupImageIdandTooltip(const Vehicles::VehicleHead& head, const bool isPlaced)
        {
            uint32_t image = 0;
            StringId tooltip = 0;
            switch (head.mode)
            {
                case TransportMode::rail:
                {
                    auto trackObj = ObjectManager::get<TrackObject>(head.trackType);
                    image = trackObj->image + (isPlaced ? TrackObj::ImageIds::kUiPickupFromTrack : TrackObj::ImageIds::kUiPlaceOnTrack);
                    tooltip = isPlaced ? StringIds::tooltip_remove_from_track : StringIds::tooltip_place_on_track;
                    break;
                }
                case TransportMode::road:
                {
                    auto roadObjId = head.trackType == 0xFF ? getGameState().defaultTrackTypeObjectId : head.trackType;
                    auto roadObj = ObjectManager::get<RoadObject>(roadObjId);
                    image = roadObj->image + (isPlaced ? 32 : 33);
                    tooltip = isPlaced ? StringIds::tooltip_remove_from_track : StringIds::tooltip_place_on_track;
                    break;
                }
                case TransportMode::air:
                {
                    image = isPlaced ? ImageIds::airport_pickup : ImageIds::airport_place;
                    tooltip = isPlaced ? StringIds::tooltip_remove_from_airport : StringIds::tooltip_place_on_airport;
                    break;
                }
                case TransportMode::water:
                {
                    auto waterObj = ObjectManager::get<WaterObject>();
                    image = waterObj->image + (isPlaced ? 58 : 59);
                    tooltip = isPlaced ? StringIds::tooltip_remove_from_water : StringIds::tooltip_place_on_dock;
                    break;
                }
            }
            return std::make_pair(image, tooltip);
        }

        // NB: not a vanilla function
        static void onClose(Window& self)
        {
            Cargo::eraseWindowState(EntityId(self.number));
            if (ToolManager::isToolActive(WindowType::vehicle, self.number))
            {
                ToolManager::toolCancel();
            }
        }

        // 0x004B26C0
        static void textInput(Window& self, [[maybe_unused]] const WidgetIndex_t callingWidget, const WidgetId id, const char* const input)
        {
            if (id != Widx::kCaption)
            {
                return;
            }

            if (strlen(input) == 0)
            {
                return;
            }

            GameCommands::setErrorTitle(StringIds::cant_rename_this_vehicle);
            GameCommands::VehicleRenameArgs args{};
            args.head = EntityId(self.number);
            std::memcpy(args.buffer, input, 36);
            args.i = 1;
            GameCommands::doCommand(args, GameCommands::Flags::apply);
            args.head = EntityId(0);
            args.i = 2;
            GameCommands::doCommand(args, GameCommands::Flags::apply);
            args.i = 0;
            GameCommands::doCommand(args, GameCommands::Flags::apply);
        }

        // 0x0050029C
        static constexpr std::array<std::array<CursorId, 2>, 6> kTypeToToolCursor = {
            {
                { { CursorId::placeTrain, CursorId::placeTrainAlt } },
                { { CursorId::placeBus, CursorId::placeBusAlt } },
                { { CursorId::placeTruck, CursorId::placeTruckAlt } },
                { { CursorId::placeTram, CursorId::placeTramAlt } },
                { { CursorId::placePlane, CursorId::placePlane } },
                { { CursorId::placeShip, CursorId::placeShip } },
            }
        };

        // 0x00427595
        static std::optional<GameCommands::VehicleWaterPlacementArgs> getVehicleWaterPlacementArgsFromCursor(const Vehicles::VehicleHead& head, const int16_t x, const int16_t y)
        {
            auto pos = ViewportInteraction::getSurfaceOrWaterLocFromUi({ x, y });
            if (!pos)
            {
                return {};
            }

            // Search 8x8 area centred on mouse pos
            const auto centerPos = *pos + World::Pos2(16, 16);
            World::Pos2 initialPos = *pos - World::toWorldSpace(World::TilePos2(4, 4));
            int32_t bestDistance = std::numeric_limits<int32_t>::max();
            World::Pos3 bestLoc{};

            for (tile_coord_t i = 0; i < 8; ++i)
            {
                for (tile_coord_t j = 0; j < 8; ++j)
                {
                    const auto loc = initialPos + World::toWorldSpace(World::TilePos2{ i, j });
                    if (!World::validCoords(loc))
                    {
                        continue;
                    }

                    auto tile = World::TileManager::get(loc);
                    for (auto& el : tile)
                    {
                        auto* elStation = el.as<StationElement>();
                        if (elStation == nullptr)
                        {
                            continue;
                        }

                        if (elStation->stationType() != StationType::docks)
                        {
                            continue;
                        }

                        if (elStation->sequenceIndex() != 0)
                        {
                            continue;
                        }

                        auto firstTile = loc - World::kOffsets[elStation->sequenceIndex()];
                        auto* dockObject = ObjectManager::get<DockObject>(elStation->objectId());
                        auto boatLoc = firstTile + World::toWorldSpace(TilePos2{ 1, 1 }) + Math::Vector::rotate(dockObject->boatPosition, elStation->rotation());

                        auto distance = Math::Vector::manhattanDistance2D(boatLoc, centerPos);
                        if (distance < bestDistance)
                        {
                            bestDistance = distance;
                            bestLoc = World::Pos3(loc.x, loc.y, elStation->baseHeight());
                        }
                    }
                }
            }

            if (bestDistance == std::numeric_limits<int32_t>::max())
            {
                return {};
            }

            GameCommands::VehicleWaterPlacementArgs args;
            args.pos = bestLoc;
            args.head = head.id;
            return { args };
        }

        static void removeBoatGhost(const Vehicles::VehicleHead& head)
        {
            // Note: don't use isPlaced as we need to know if its a ghost
            // consider creating isGhostPlaced
            if (head.tileX != -1 && head.has38Flags(Vehicles::Flags38::isGhost))
            {
                GameCommands::VehiclePickupWaterArgs gcArgs{};
                gcArgs.head = head.id;
                auto flags = GameCommands::Flags::apply | GameCommands::Flags::noErrorWindow | GameCommands::Flags::ghost;
                GameCommands::doCommand(gcArgs, flags);
            }
            _ghostTrackProgress = -1;
        }

        // 0x004B2B9E
        static void pickupToolUpdateWater(const Vehicles::VehicleHead& head, const int16_t x, const int16_t y)
        {
            auto placementArgs = getVehicleWaterPlacementArgsFromCursor(head, x, y);

            if (!placementArgs)
            {
                removeBoatGhost(head);
                return;
            }

            if (_ghostTrackProgress == 0 && _ghostVehiclePos == placementArgs->pos)
            {
                return;
            }
            _ghostVehiclePos = placementArgs->pos;
            removeBoatGhost(head);
            if (GameCommands::doCommand(*placementArgs, GameCommands::Flags::apply | GameCommands::Flags::ghost | GameCommands::Flags::noErrorWindow) != GameCommands::kFailure)
            {
                _ghostTrackProgress = 0;
            }
        }

        // 0x00426F0B
        static std::optional<GameCommands::VehicleAirPlacementArgs> getVehicleAirPlacementArgsFromCursor(const Vehicles::VehicleHead& head, const int16_t x, const int16_t y)
        {
            auto res = ViewportInteraction::getMapCoordinatesFromPos(x, y, ~ViewportInteraction::InteractionItemFlags::station);
            World::StationElement* elStation = nullptr;
            if (res.first.type == ViewportInteraction::InteractionItem::airport)
            {
                elStation = static_cast<World::TileElementEntry*>(res.first.object)->as<World::StationElement>();
            }
            else
            {
                res = ViewportInteraction::getMapCoordinatesFromPos(x, y, ~(ViewportInteraction::InteractionItemFlags::surface | ViewportInteraction::InteractionItemFlags::water));
                const auto& interaction = res.first;
                if (interaction.type == ViewportInteraction::InteractionItem::noInteraction)
                {
                    return {};
                }

                bool stationFound = false;
                for (auto& station : StationManager::stations())
                {
                    if ((station.flags & StationFlags::flag_6) == StationFlags::none)
                    {
                        continue;
                    }
                    if ((station.flags & StationFlags::flag_5) != StationFlags::none)
                    {
                        continue;
                    }

                    if (std::abs(interaction.pos.x - station.airportStartPos.x) > 5 * World::kTileSize)
                    {
                        continue;
                    }
                    if (std::abs(interaction.pos.y - station.airportStartPos.y) > 5 * World::kTileSize)
                    {
                        continue;
                    }

                    auto tile = TileManager::get(station.airportStartPos);
                    for (auto& el : tile)
                    {
                        elStation = el.as<StationElement>();
                        if (elStation == nullptr)
                        {
                            continue;
                        }

                        if (elStation->isAiAllocated() || elStation->isGhost())
                        {
                            continue;
                        }

                        if (elStation->stationType() != StationType::airport)
                        {
                            continue;
                        }

                        stationFound = true;
                        break;
                    }
                }

                if (!stationFound)
                {
                    return {};
                }
            }

            if (elStation == nullptr || elStation->isAiAllocated() || elStation->isGhost())
            {
                return {};
            }

            GameCommands::VehicleAirPlacementArgs placementArgs;
            placementArgs.stationId = elStation->stationId();
            placementArgs.head = head.id;

            auto* airportObj = ObjectManager::get<AirportObject>(elStation->objectId());
            const auto movementNodes = airportObj->getMovementNodes();

            int32_t bestDistance = std::numeric_limits<int32_t>::max();
            uint8_t bestNode = 0;
            auto* vpOwner = WindowManager::findWindowForViewport(res.second);
            const auto vpOffset = vpOwner != nullptr ? vpOwner->position() : Ui::Point{};
            // TODO: Use std::ranges::reverse_view
            for (auto node = airportObj->numMovementNodes - 1; node > -1; node--)
            {
                const auto& movementNode = movementNodes[node];
                if (!movementNode.hasFlags(AirportMovementNodeFlags::terminal))
                {
                    continue;
                }
                auto nodeLoc = getAirportMovementNodeLoc(placementArgs.stationId, node);
                if (!nodeLoc)
                {
                    continue;
                }

                auto viewPos = World::gameToScreen(*nodeLoc, res.second->getRotation());
                auto uiPos = res.second->viewportToWindow(viewPos) + vpOffset;
                auto distance = Math::Vector::manhattanDistance2D(uiPos, Point{ x, y });
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestNode = node;
                }
            }

            if (bestDistance == std::numeric_limits<int32_t>::max())
            {
                return {};
            }

            placementArgs.airportNode = bestNode;
            return { placementArgs };
        }

        static void removeAirplaneGhost(const Vehicles::VehicleHead& head)
        {
            // Note: don't use isPlaced as we need to know if its a ghost
            // consider creating isGhostPlaced
            if (head.tileX != -1 && head.has38Flags(Vehicles::Flags38::isGhost))
            {
                GameCommands::VehiclePickupAirArgs gcArgs{};
                gcArgs.head = head.id;
                auto flags = GameCommands::Flags::apply | GameCommands::Flags::noErrorWindow | GameCommands::Flags::ghost;
                GameCommands::doCommand(gcArgs, flags);
            }
            _ghostAirportStationId = StationId::null;
        }

        // 0x004B2AFA
        static void pickupToolUpdateAir(const Vehicles::VehicleHead& head, const int16_t x, const int16_t y)
        {
            auto placementArgs = getVehicleAirPlacementArgsFromCursor(head, x, y);
            if (!placementArgs)
            {
                removeAirplaneGhost(head);
                return;
            }

            if (_ghostAirportStationId != StationId::null && _ghostAirportStationId == placementArgs->stationId && _ghostAirportNode == placementArgs->airportNode)
            {
                return;
            }

            removeAirplaneGhost(head);
            if (GameCommands::doCommand(*placementArgs, GameCommands::Flags::apply | GameCommands::Flags::ghost | GameCommands::Flags::noErrorWindow) != GameCommands::kFailure)
            {
                _ghostAirportNode = placementArgs->airportNode;
                _ghostAirportStationId = placementArgs->stationId;
            }
        }

        // 0x004A43E4
        static uint16_t getRoadProgressAtCursor(const Point& cursorLoc, Ui::Viewport& viewport, const RoadElement& roadElement, const World::Pos3& loc)
        {
            // Get the coordinates of the first tile of the possibly multi-tile road
            const auto& roadDataArr = World::TrackData::getRoadPiece(roadElement.roadId());
            const auto& roadData = roadDataArr[roadElement.sequenceIndex()];
            auto roadOffset2 = Math::Vector::rotate(World::Pos2(roadData.x, roadData.y), roadElement.rotation());
            auto roadOffset = World::Pos3(roadOffset2.x, roadOffset2.y, roadData.z);
            auto roadFirstTile = loc - roadOffset;

            // Get the movement info for this specific road id
            uint16_t trackAndDirection = roadElement.rotation() | (roadElement.roadId() << 3);
            const auto moveInfoArr = World::TrackData::getRoadPlacementSubPositon(trackAndDirection);

            // This iterates the movement info trying to find the distance along the road that is as close as possible
            // to the cursors location.
            int32_t bestDistance = std::numeric_limits<int32_t>::max();
            uint16_t bestProgress = 0;
            auto* vpOwner = WindowManager::findWindowForViewport(&viewport);
            const auto vpOffset = vpOwner != nullptr ? vpOwner->position() : Ui::Point{};
            for (const auto& moveInfo : moveInfoArr)
            {
                auto potentialLoc = roadFirstTile + moveInfo.loc;
                auto viewPos = World::gameToScreen(potentialLoc, viewport.getRotation());
                auto uiPos = viewport.viewportToWindow(viewPos) + vpOffset;
                auto distance = Math::Vector::manhattanDistance2D(uiPos, cursorLoc);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestProgress = std::distance(&*moveInfoArr.begin(), &moveInfo);
                }
            }
            return bestProgress;
        }

        // 0x00478415
        static std::optional<GameCommands::VehiclePlacementArgs> getRoadAtCursor(const int16_t x, const int16_t y)
        {
            auto [interaction, viewport] = ViewportInteraction::getMapCoordinatesFromPos(x, y, ~ViewportInteraction::InteractionItemFlags::roadAndTram);
            if (interaction.type != ViewportInteraction::InteractionItem::road)
            {
                return {};
            }

            // Get the best progress along the road relative to the cursor
            auto* roadElement = static_cast<World::TileElementEntry*>(interaction.object)->as<World::RoadElement>();
            if (roadElement == nullptr)
            {
                return {};
            }

            World::Pos3 loc(interaction.pos.x, interaction.pos.y, roadElement->baseHeight());
            auto progress = getRoadProgressAtCursor({ x, y }, *viewport, *roadElement, loc);

            // Get the coordinates of the first tile of the possibly multi-tile road
            const auto& roadDataArr = World::TrackData::getRoadPiece(roadElement->roadId());
            const auto& roadData = roadDataArr[roadElement->sequenceIndex()];
            auto roadOffset2 = Math::Vector::rotate(World::Pos2(roadData.x, roadData.y), roadElement->rotation());
            auto roadOffset = World::Pos3(roadOffset2.x, roadOffset2.y, roadData.z);
            auto roadFirstTile = loc - roadOffset;

            GameCommands::VehiclePlacementArgs placementArgs;
            placementArgs.pos = roadFirstTile;
            placementArgs.trackProgress = progress;
            placementArgs.trackAndDirection = roadElement->rotation() | (roadElement->roadId() << 3);
            return { placementArgs };
        }

        // 0x00479707
        static std::optional<GameCommands::VehiclePlacementArgs> getVehicleRoadPlacementArgsFromCursor(const Vehicles::VehicleHead& head, const int16_t x, const int16_t y)
        {
            auto placementArgs = getRoadAtCursor(x, y);
            if (!placementArgs)
            {
                return {};
            }

            placementArgs->head = head.id;
            const auto moveInfoArr = World::TrackData::getRoadPlacementSubPositon(placementArgs->trackAndDirection);
            const auto& moveInfo = moveInfoArr[placementArgs->trackProgress];

            uint8_t unkYaw = moveInfo.yaw + (WindowManager::getCurrentRotation() << 4);
            unkYaw -= 0x37;

            if (getGameState().pickupDirection != 0)
            {
                unkYaw -= 0x20;
            }
            unkYaw &= 0x3F;
            if (unkYaw <= 0x20)
            {
                const auto& unkItem = TrackData::getUnkRoad(placementArgs->trackAndDirection);
                placementArgs->pos += unkItem.pos;
                if (unkItem.rotationEnd < 12)
                {
                    placementArgs->pos -= World::Pos3{ World::kRotationOffset[unkItem.rotationEnd], 0 };
                }
                placementArgs->trackProgress = std::max<uint16_t>(static_cast<uint16_t>(moveInfoArr.size()) - placementArgs->trackProgress, 0);
                if (placementArgs->trackProgress >= moveInfoArr.size())
                {
                    placementArgs->trackProgress = static_cast<uint16_t>(moveInfoArr.size()) - 1;
                }
                placementArgs->trackAndDirection ^= (1 << 2);
            }
            return placementArgs;
        }

        // 0x004A43E4
        static uint16_t getTrackProgressAtCursor(const Point& cursorLoc, Ui::Viewport& viewport, const TrackElement& trackElement, const World::Pos3& loc)
        {
            // Get the coordinates of the first tile of the possibly multi-tile track
            const auto& trackDataArr = World::TrackData::getTrackPiece(trackElement.trackId());
            const auto& trackData = trackDataArr[trackElement.sequenceIndex()];
            auto trackOffset2 = Math::Vector::rotate(World::Pos2(trackData.x, trackData.y), trackElement.rotation());
            auto trackOffset = World::Pos3(trackOffset2.x, trackOffset2.y, trackData.z);
            auto trackFirstTile = loc - trackOffset;

            // Get the movement info for this specific track id
            uint16_t trackAndDirection = trackElement.rotation() | (trackElement.trackId() << 3);
            const auto moveInfoArr = World::TrackData::getTrackSubPositon(trackAndDirection);

            // This iterates the movement info trying to find the distance along the track that is as close as possible
            // to the cursors location.
            int32_t bestDistance = std::numeric_limits<int32_t>::max();
            uint16_t bestProgress = 0;
            auto* vpOwner = WindowManager::findWindowForViewport(&viewport);
            const auto vpOffset = vpOwner != nullptr ? vpOwner->position() : Ui::Point{};
            for (const auto& moveInfo : moveInfoArr)
            {
                auto potentialLoc = trackFirstTile + moveInfo.loc;
                auto viewPos = World::gameToScreen(potentialLoc, viewport.getRotation());
                auto uiPos = viewport.viewportToWindow(viewPos) + vpOffset;
                auto distance = Math::Vector::manhattanDistance2D(uiPos, cursorLoc);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestProgress = std::distance(&*moveInfoArr.begin(), &moveInfo);
                }
            }
            return bestProgress;
        }

        // 0x004A40C5
        static std::optional<GameCommands::VehiclePlacementArgs> getTrackAtCursor(const int16_t x, const int16_t y)
        {
            auto [interaction, viewport] = ViewportInteraction::getMapCoordinatesFromPos(x, y, ~ViewportInteraction::InteractionItemFlags::track);
            if (interaction.type != ViewportInteraction::InteractionItem::track)
            {
                return {};
            }

            // Get the best progress along the track relative to the cursor
            auto* trackElement = static_cast<World::TileElementEntry*>(interaction.object)->as<World::TrackElement>();
            if (trackElement == nullptr)
            {
                return {};
            }

            World::Pos3 loc(interaction.pos.x, interaction.pos.y, trackElement->baseHeight());
            auto progress = getTrackProgressAtCursor({ x, y }, *viewport, *trackElement, loc);

            // Get the coordinates of the first tile of the possibly multi-tile road
            const auto& trackDataArr = World::TrackData::getTrackPiece(trackElement->trackId());
            const auto& trackData = trackDataArr[trackElement->sequenceIndex()];
            auto trackOffset2 = Math::Vector::rotate(World::Pos2(trackData.x, trackData.y), trackElement->rotation());
            auto trackOffset = World::Pos3(trackOffset2.x, trackOffset2.y, trackData.z);
            auto trackFirstTile = loc - trackOffset;

            GameCommands::VehiclePlacementArgs placementArgs;
            placementArgs.pos = trackFirstTile;
            placementArgs.trackProgress = progress;
            placementArgs.trackAndDirection = trackElement->rotation() | (trackElement->trackId() << 3);
            return { placementArgs };
        }

        // 0x004B6444
        static std::optional<GameCommands::VehiclePlacementArgs> getVehicleRailPlacementArgsFromCursor(const Vehicles::VehicleHead& head, const int16_t x, const int16_t y)
        {
            auto placementArgs = getTrackAtCursor(x, y);
            if (!placementArgs)
            {
                return {};
            }

            placementArgs->head = head.id;
            const auto moveInfoArr = World::TrackData::getTrackSubPositon(placementArgs->trackAndDirection);
            const auto& moveInfo = moveInfoArr[placementArgs->trackProgress];

            uint8_t unkYaw = moveInfo.yaw + (WindowManager::getCurrentRotation() << 4);
            unkYaw -= 0x37;
            if (getGameState().pickupDirection != 0)
            {
                unkYaw -= 0x20;
            }
            unkYaw &= 0x3F;
            if (unkYaw <= 0x20)
            {
                const auto& unkItem = TrackData::getUnkTrack(placementArgs->trackAndDirection);
                placementArgs->pos += unkItem.pos;
                if (unkItem.rotationEnd < 12)
                {
                    placementArgs->pos -= World::Pos3{ World::kRotationOffset[unkItem.rotationEnd], 0 };
                }
                placementArgs->trackProgress = std::max<uint16_t>(static_cast<uint16_t>(moveInfoArr.size()) - placementArgs->trackProgress, 0);
                if (placementArgs->trackProgress >= moveInfoArr.size())
                {
                    placementArgs->trackProgress = static_cast<uint16_t>(moveInfoArr.size()) - 1;
                }
                placementArgs->trackAndDirection ^= (1 << 2);
            }
            return placementArgs;
        }

        static void removeLandGhost(const Vehicles::VehicleHead& head)
        {
            // Note: don't use isPlaced as we need to know if its a ghost
            // consider creating isGhostPlaced
            if (head.tileX != -1 && head.has38Flags(Vehicles::Flags38::isGhost))
            {
                GameCommands::VehiclePickupArgs args{};
                args.head = head.id;
                GameCommands::doCommand(args, GameCommands::Flags::apply | GameCommands::Flags::noErrorWindow | GameCommands::Flags::ghost);
            }
            _ghostTrackProgress = -1;
        }

        // 0x004B2A1D
        template<typename GetPlacementArgsFunc>
        static void pickupToolUpdateLand(const Vehicles::VehicleHead& head, const int16_t x, const int16_t y, GetPlacementArgsFunc&& getPlacementArgs)
        {
            auto placementArgs = getPlacementArgs(head, x, y);
            if (!placementArgs)
            {
                removeLandGhost(head);
                return;
            }

            if (_ghostTrackProgress != -1 && _ghostLandTrackAndDirection == placementArgs->trackAndDirection && _ghostVehiclePos == placementArgs->pos && _ghostTrackProgress == placementArgs->trackProgress)
            {
                return;
            }

            removeLandGhost(head);
            if (GameCommands::doCommand(*placementArgs, GameCommands::Flags::apply | GameCommands::Flags::ghost | GameCommands::Flags::noErrorWindow) != GameCommands::kFailure)
            {
                _ghostLandTrackAndDirection = placementArgs->trackAndDirection;
                _ghostVehiclePos = placementArgs->pos;
                _ghostTrackProgress = placementArgs->trackProgress;
            }
        }

        // 0x004B29C0
        static void pickupToolUpdate(Window& self, const int16_t x, const int16_t y)
        {
            auto* head = getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            ToolManager::setToolCursor(kTypeToToolCursor[static_cast<uint8_t>(head->vehicleType)][getGameState().pickupDirection != 0 ? 1 : 0]);

            switch (head->mode)
            {
                case TransportMode::rail:
                    pickupToolUpdateLand(*head, x, y, getVehicleRailPlacementArgsFromCursor);
                    break;
                case TransportMode::road:
                    pickupToolUpdateLand(*head, x, y, getVehicleRoadPlacementArgsFromCursor);
                    break;
                case TransportMode::air:
                    pickupToolUpdateAir(*head, x, y);
                    break;
                case TransportMode::water:
                    pickupToolUpdateWater(*head, x, y);
                    break;
            }
        }

        // 0x004B2D8A
        static void pickupToolError(const Vehicles::VehicleHead& head)
        {
            auto args = FormatArguments::common();
            args.skip(6);
            args.push(head.name);
            args.push(head.ordinalNumber);
            Error::open(StringIds::cant_place_string_id_here, StringIds::null);
        }

        static void pickupToolPlacementCommandCallback(uint32_t gameCommandResult, Window& self, EntityId vehicleHead)
        {
            if (gameCommandResult == GameCommands::kFailure)
            {
                return;
            }

            if (Input::hasKeyModifier(Input::KeyModifier::shift))
            {
                VehicleChangeRunningModeArgs args{};
                args.head = vehicleHead;
                args.mode = VehicleChangeRunningModeArgs::Mode::startVehicle;
                GameCommands::doCommand(args, GameCommands::Flags::apply);
            }

            ToolManager::toolCancel();
            self.callOnMouseUp(Common::widx::tabMain, self.widgets[Common::widx::tabMain].id);
        }

        // 0x004B2E18
        static void pickupToolDownAir(Window& self, const Vehicles::VehicleHead& head, const int16_t x, const int16_t y)
        {
            auto placementArgs = getVehicleAirPlacementArgsFromCursor(head, x, y);
            if (!placementArgs)
            {
                pickupToolError(head);
                return;
            }

            if (_ghostAirportStationId == placementArgs->stationId && _ghostAirportNode == placementArgs->airportNode)
            {
                if (head.tileX != -1 && head.has38Flags(Vehicles::Flags38::isGhost))
                {
                    // Will convert inplace vehicle into non ghost
                    placementArgs->convertGhost = true;
                }
            }
            if (!placementArgs->convertGhost)
            {
                removeAirplaneGhost(head);
            }
            auto args = FormatArguments::common();
            args.skip(6);
            args.push(head.name);
            args.push(head.ordinalNumber);
            GameCommands::setErrorTitle(StringIds::cant_place_string_id_here);

            auto result = GameCommands::doCommand(*placementArgs, GameCommands::Flags::apply);
            pickupToolPlacementCommandCallback(result, self, head.head);
        }

        // 0x004B2F1C
        static void pickupToolDownWater(Window& self, const Vehicles::VehicleHead& head, const int16_t x, const int16_t y)
        {
            auto placementArgs = getVehicleWaterPlacementArgsFromCursor(head, x, y);
            if (!placementArgs)
            {
                pickupToolError(head);
                return;
            }

            if (_ghostTrackProgress == 0 && _ghostVehiclePos == placementArgs->pos)
            {
                if (head.tileX != -1 && head.has38Flags(Vehicles::Flags38::isGhost))
                {
                    // Will convert inplace vehicle into non ghost
                    placementArgs->convertGhost = true;
                }
            }
            if (!placementArgs->convertGhost)
            {
                removeBoatGhost(head);
            }
            auto args = FormatArguments::common();
            args.skip(6);
            args.push(head.name);
            args.push(head.ordinalNumber);
            GameCommands::setErrorTitle(StringIds::cant_place_string_id_here);

            auto result = GameCommands::doCommand(*placementArgs, GameCommands::Flags::apply);
            pickupToolPlacementCommandCallback(result, self, head.head);
        }

        // 0x004B2C95
        template<typename GetPlacementArgsFunc>
        static void pickupToolDownLand(Window& self, const Vehicles::VehicleHead& head, const int16_t x, const int16_t y, GetPlacementArgsFunc&& getPlacementArgs)
        {
            auto placementArgs = getPlacementArgs(head, x, y);
            if (!placementArgs)
            {
                pickupToolError(head);
                return;
            }

            if (_ghostLandTrackAndDirection == placementArgs->trackAndDirection && _ghostVehiclePos == placementArgs->pos && _ghostTrackProgress == placementArgs->trackProgress)
            {
                if (head.tileX != -1 && head.has38Flags(Vehicles::Flags38::isGhost))
                {
                    // Will convert inplace vehicle into non ghost
                    placementArgs->convertGhost = true;
                }
            }
            if (!placementArgs->convertGhost)
            {
                removeLandGhost(head);
            }
            auto args = FormatArguments::common();
            args.skip(6);
            args.push(head.name);
            args.push(head.ordinalNumber);
            GameCommands::setErrorTitle(StringIds::cant_place_string_id_here);

            auto result = GameCommands::doCommand(*placementArgs, GameCommands::Flags::apply);
            pickupToolPlacementCommandCallback(result, self, head.head);
        }

        // 0x004B2C74
        static void pickupToolDown(Window& self, const int16_t x, const int16_t y)
        {
            auto* head = getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            switch (head->mode)
            {
                case TransportMode::rail:
                    pickupToolDownLand(self, *head, x, y, getVehicleRailPlacementArgsFromCursor);
                    break;
                case TransportMode::road:
                    pickupToolDownLand(self, *head, x, y, getVehicleRoadPlacementArgsFromCursor);
                    break;
                case TransportMode::air:
                    pickupToolDownAir(self, *head, x, y);
                    break;
                case TransportMode::water:
                    pickupToolDownWater(self, *head, x, y);
                    break;
            }
        }

        // 0x004B3035
        static void pickupToolAbort(Window& self)
        {
            // TODO: refactor to use removeAirplaneGhost family of functions
            auto* head = EntityManager::get<Vehicles::VehicleHead>(EntityId(self.number));
            if (head == nullptr)
            {
                return;
            }
            if (head->tileX == -1 || !head->has38Flags(Vehicles::Flags38::isGhost))
            {
                self.invalidate();
                return;
            }

            switch (head->mode)
            {
                case TransportMode::rail:
                case TransportMode::road:
                {
                    GameCommands::VehiclePickupArgs args{};
                    args.head = head->id;
                    auto flags = GameCommands::Flags::apply | GameCommands::Flags::noErrorWindow | GameCommands::Flags::ghost;
                    GameCommands::doCommand(args, flags);
                    break;
                }
                case TransportMode::air:
                {
                    GameCommands::VehiclePickupAirArgs gcArgs{};
                    gcArgs.head = head->id;
                    auto flags = GameCommands::Flags::apply | GameCommands::Flags::noErrorWindow | GameCommands::Flags::ghost;
                    GameCommands::doCommand(gcArgs, flags);
                    break;
                }
                case TransportMode::water:
                {
                    GameCommands::VehiclePickupWaterArgs gcArgs{};
                    gcArgs.head = head->id;
                    auto flags = GameCommands::Flags::apply | GameCommands::Flags::noErrorWindow | GameCommands::Flags::ghost;
                    GameCommands::doCommand(gcArgs, flags);
                    break;
                }
            }
            self.invalidate();
        }

        // 0x004B2680
        static void renameVehicle(Window& self, WidgetIndex_t widgetIndex)
        {
            auto vehicle = getVehicle(self);
            if (vehicle != nullptr)
            {
                FormatArguments args{};
                args.push(StringIds::getVehicleType(vehicle->vehicleType)); // 0
                args.skip(6);
                args.push(StringIds::getVehicleType(vehicle->vehicleType)); // 8

                FormatArgumentsBuffer buffer{};
                auto args2 = FormatArguments(buffer);
                args2.push(vehicle->ordinalNumber);
                TextInput::openTextInput(&self, StringIds::title_name_vehicle, StringIds::prompt_enter_new_vehicle_name, vehicle->name, widgetIndex, args2);
            }
        }

        // 0x004B2566
        static void switchTab(Window& self, WidgetIndex_t widgetIndex)
        {
            if (self.currentTab == tabRoute - tabMain)
            {
                Route::resetSharedOrderDropdown(EntityId(self.number));
            }
            ToolManager::toolCancel(self.type, self.number);
            TextInput::sub_4CE6C9(self.type, self.number);

            self.currentTab = widgetIndex - Common::widx::tabMain;
            self.frameNo = 0;
            self.flags &= ~WindowFlags::maximised;
            self.var_85C = -1;
            self.viewportRemove(0);

            auto tabInfo = tabInformationByTabOffset[widgetIndex - widx::tabMain];
            self.holdableWidgets = tabInfo.holdableWidgets;
            self.eventHandlers = &tabInfo.events;
            self.activatedWidgets = 0;
            self.setWidgets(tabInfo.widgets);
            self.disabledWidgets = 0;
            Main::resetDisabledWidgets(self);

            self.invalidate();
            self.rowHover = -1;
            self.orderTableIndex = -1;

            self.setSizeBounds(tabInfo.minSize, tabInfo.maxSize);
            self.callOnResize();
            self.callPrepareDraw();
            self.initScrollWidgets();
            self.invalidate();
            self.moveInsideScreenEdges();

            if (widgetIndex == Common::tabDetails)
            {
                auto company = CompanyManager::get(self.owner);
                self.widgets[Details::widx::paintColourPrimary].image = Widget::kImageIdColourSet | ImageId::fromUInt32(ImageIds::colour_swatch_recolourable).withPrimary(company->mainColours.primary).toUInt32();
                self.widgets[Details::widx::paintColourSecondary].image = Widget::kImageIdColourSet | ImageId::fromUInt32(ImageIds::colour_swatch_recolourable).withPrimary(company->mainColours.secondary).toUInt32();
            }
        }

        // 0x004B1E94
        static void setCaptionEnableState(Window& self)
        {
            self.disabledWidgets &= ~(1ULL << widx::caption);
            auto head = getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            if (head->owner != CompanyManager::getControllingId())
            {
                self.disabledWidgets |= (1ULL << widx::caption);
            }
        }

        // 0x004B28E2
        static void onPickup(Window& self, const WidgetIndex_t pickupWidx)
        {
            self.invalidate();
            auto head = getVehicle(self);
            if (head == nullptr)
            {
                return;
            }
            if (!head->isPlaced())
            {
                CursorId cursor = kTypeToToolCursor[static_cast<uint8_t>(head->vehicleType)][getGameState().pickupDirection != 0 ? 1 : 0];
                if (ToolManager::toolSet(self, pickupWidx, cursor))
                {
                    _ghostTrackProgress = -1;
                }
                return;
            }

            Vehicles::Vehicle train(*head);
            EntityId viewportFollowEntity = train.veh2->id;
            auto main = Ui::WindowManager::getMainWindow();
            if (Windows::Main::viewportIsFocusedOnEntity(*main, viewportFollowEntity))
            {
                Windows::Main::viewportUnfocusFromEntity(*main);
            }

            GameCommands::setErrorTitle(StringIds::cant_remove_string_id);
            FormatArguments args{};
            args.skip(6);
            args.push(head->name);
            args.push(head->ordinalNumber);

            bool success = false;
            switch (head->mode)
            {
                case TransportMode::rail:
                case TransportMode::road:
                {
                    GameCommands::VehiclePickupArgs gcArgs{};
                    gcArgs.head = head->id;
                    success = GameCommands::doCommand(gcArgs, GameCommands::Flags::apply) != GameCommands::kFailure;
                    break;
                }
                case TransportMode::air:
                {
                    GameCommands::VehiclePickupAirArgs gcArgs{};
                    gcArgs.head = head->id;
                    success = GameCommands::doCommand(gcArgs, GameCommands::Flags::apply) != GameCommands::kFailure;
                    break;
                }
                case TransportMode::water:
                {
                    GameCommands::VehiclePickupWaterArgs gcArgs{};
                    gcArgs.head = head->id;
                    success = GameCommands::doCommand(gcArgs, GameCommands::Flags::apply) != GameCommands::kFailure;
                    break;
                }
            }
            if (success)
            {
                self.callOnMouseUp(widx::tabDetails, self.widgets[widx::tabDetails].id);
            }
        }

        static size_t getNumCars(Ui::Window& self)
        {
            auto* head = getVehicle(self);
            if (head == nullptr)
            {
                return 0;
            }
            Vehicles::Vehicle train(*head);

            if (train.cars.empty())
            {
                return 0;
            }

            return train.cars.size();
        }

        // 0x004B5CC1
        static std::optional<Vehicles::Car> getCarFromScrollView(Window& self, const int16_t y)
        {
            auto* head = getVehicle(self);
            if (head == nullptr)
            {
                return {};
            }
            Vehicles::Vehicle train(*head);

            auto heightOffset = y;
            for (auto& car : train.cars)
            {
                heightOffset -= self.rowHeight;
                if (heightOffset <= 0)
                {
                    return { car };
                }
            }
            return {};
        }

        struct TabIcons
        {
            uint32_t image;
            uint8_t frameSpeed;
        };

        static constexpr auto kTabIconByVehicleType = Utility::buildLookupTable<VehicleType, TabIcons>({
            { VehicleType::train, { InterfaceSkin::ImageIds::tab_vehicle_train_frame0, 1 } },
            { VehicleType::bus, { InterfaceSkin::ImageIds::tab_vehicle_bus_frame0, 1 } },
            { VehicleType::truck, { InterfaceSkin::ImageIds::tab_vehicle_truck_frame0, 1 } },
            { VehicleType::tram, { InterfaceSkin::ImageIds::tab_vehicle_tram_frame0, 1 } },
            { VehicleType::aircraft, { InterfaceSkin::ImageIds::tab_vehicle_aircraft_frame0, 2 } },
            { VehicleType::ship, { InterfaceSkin::ImageIds::tab_vehicle_ship_frame0, 3 } },
        });

        // 0x004B5F0D
        static void drawTabs(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            auto skin = OpenLoco::ObjectManager::get<InterfaceSkinObject>();

            auto vehicle = Common::getVehicle(self);
            if (vehicle == nullptr)
            {
                return;
            }
            auto vehicleType = vehicle->vehicleType;

            auto mainTab = kTabIconByVehicleType.at(vehicleType);
            int frame = 0;
            if (self.currentTab == 0)
            {
                frame = (self.frameNo >> mainTab.frameSpeed) & 0x7;
            }

            Widget::drawTab(
                self,
                drawingCtx,
                Gfx::recolour(skin->img + mainTab.image + frame, CompanyManager::getCompanyColour(self.owner)),
                widx::tabMain);

            frame = 0;
            if (self.currentTab == 1)
            {
                frame = (self.frameNo >> 1) & 0xF;
            }
            Widget::drawTab(
                self,
                drawingCtx,
                skin->img + InterfaceSkin::ImageIds::tab_wrench_frame0 + frame,
                widx::tabDetails);

            frame = 0;
            if (self.currentTab == 2)
            {
                frame = (self.frameNo >> 3) & 0x3;
            }
            Widget::drawTab(
                self,
                drawingCtx,
                skin->img + InterfaceSkin::ImageIds::tab_cargo_delivered_frame0 + frame,
                widx::tabCargo);

            frame = 0;
            if (self.currentTab == 4)
            {
                frame = (self.frameNo >> 4) & 0x3;
            }
            Widget::drawTab(
                self,
                drawingCtx,
                skin->img + InterfaceSkin::ImageIds::tab_routes_frame_0 + frame,
                widx::tabRoute);

            frame = 0;
            if (self.currentTab == 3)
            {
                frame = (self.frameNo >> 1) & 0xF;
            }
            Widget::drawTab(
                self,
                drawingCtx,
                skin->img + InterfaceSkin::ImageIds::tab_finances_frame0 + frame,
                widx::tabFinances);
        }
    }

    // 0x004B949C
    bool rotate()
    {
        if (ToolManager::isToolActive(WindowType::vehicle))
        {
            if (ToolManager::getToolWidgetIndex() == Main::widx::pickup || ToolManager::getToolWidgetIndex() == Details::widx::pickup)
            {
                getGameState().pickupDirection = getGameState().pickupDirection ^ 1;
                return true;
            }
        }

        return false;
    }

    // 0x004B944A
    bool cancelVehicleTools()
    {
        if (ToolManager::isToolActive(WindowType::vehicle))
        {
            auto* w = WindowManager::find(WindowType::vehicle, ToolManager::getToolWindowNumber());
            if (w->currentTab == (Common::widx::tabMain - Common::widx::tabMain))
            {
                w->callOnMouseUp(Common::widx::tabDetails, w->widgets[Common::widx::tabDetails].id);
                return true;
            }
            else if (w->currentTab == (Common::widx::tabRoute - Common::widx::tabMain))
            {
                w->callOnMouseUp(Common::widx::tabMain, w->widgets[Common::widx::tabMain].id);
                return true;
            }
        }
        return false;
    }
}
