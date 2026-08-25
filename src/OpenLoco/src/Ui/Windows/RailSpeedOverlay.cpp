// SPDX-License-Identifier: MIT
#include "Ui/Windows/RailSpeedOverlay.h"

#include "Graphics/DrawingContext.h"
#include "Graphics/ImageIds.h"
#include "Graphics/TextRenderer.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/StringIds.h"
#include "Map/TileElementEntry.h"
#include "Map/Track/TrackData.h"
#include "Map/TrackElement.h"
#include "Objects/InterfaceSkinObject.h"
#include "Objects/ObjectManager.h"
#include "Ui/ViewportInteraction.h"
#include "Ui/Widgets/CaptionWidget.h"
#include "Ui/Widgets/FrameWidget.h"
#include "Ui/Widgets/ImageButtonWidget.h"
#include "Ui/Widgets/PanelWidget.h"
#include "Ui/WindowManager.h"
#include "Ui/Windows/CargoFlowOverlay.h"
#include "Vehicles/RailTraffic.h"
#include "Viewport.hpp"
#include <OpenLoco/Math/Vector.hpp>
#include <algorithm>
#include <vector>

namespace OpenLoco::Ui::Windows::RailSpeedOverlay
{
    namespace
    {
        constexpr Ui::Size kWindowSize = { 252, 62 };
        constexpr int16_t kLegendLeft = 46;
        constexpr int16_t kLegendTop = 34;
        constexpr int16_t kLegendCellWidth = 13;

        namespace Widx
        {
            constexpr WidgetId kFrame{ "frame" };
            constexpr WidgetId kCaption{ "caption" };
            constexpr WidgetId kCloseButton{ "close_button" };
            constexpr WidgetId kPanel{ "panel" };
        }

        static constexpr auto kWidgets = makeWidgets(
            Widgets::Frame(Widx::kFrame, { 0, 0 }, kWindowSize, WindowColour::primary),
            Widgets::Caption(Widx::kCaption, { 1, 1 }, { kWindowSize.width - 2, 13 }, Widgets::Caption::Style::whiteText, WindowColour::primary, StringIds::rail_speed_overlay),
            Widgets::ImageButton(Widx::kCloseButton, { kWindowSize.width - 15, 2 }, { 13, 13 }, WindowColour::primary, ImageIds::close_button, StringIds::tooltip_close_window),
            Widgets::Panel(Widx::kPanel, { 0, 15 }, { kWindowSize.width, kWindowSize.height - 15 }, WindowColour::secondary));

        uint32_t _historyRevision{};
        uint8_t _refreshTicks{};
        SpeedThresholds _speedThresholds{};

        void invalidateMainViewport()
        {
            if (auto* main = WindowManager::getMainWindow(); main != nullptr)
            {
                main->invalidate();
            }
        }

        uint64_t getPercentileValue(const Speed16 speed)
        {
            return static_cast<uint64_t>(std::max<int32_t>(0, speed.getRaw())) + 1;
        }

        void refreshSpeedThresholds()
        {
            const auto speeds = Vehicles::RailTraffic::getAverageSpeeds();
            _speedThresholds = calculateSpeedPercentileThresholds(speeds);
        }

        void onClose(Window&)
        {
            invalidateMainViewport();
        }

        void onMouseUp(Window& self, WidgetIndex_t, const WidgetId id)
        {
            if (id == Widx::kCloseButton)
            {
                WindowManager::close(&self);
            }
        }

        void onUpdate(Window&)
        {
            if (++_refreshTicks < 16)
            {
                return;
            }
            _refreshTicks = 0;
            const auto revision = Vehicles::RailTraffic::getHistoryRevision();
            if (_historyRevision != revision)
            {
                _historyRevision = revision;
                refreshSpeedThresholds();
                invalidateMainViewport();
            }
        }

        void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            self.draw(drawingCtx);
            auto tr = Gfx::TextRenderer(drawingCtx);
            tr.setCurrentFont(Gfx::Font::small);
            const auto colour = self.getColour(WindowColour::secondary).opaque();
            tr.drawStringCentred({ kWindowSize.width / 2, 20 }, colour, StringIds::rail_speed_overlay_description);
            for (uint8_t i = 0; i < kBucketCount; ++i)
            {
                const auto left = kLegendLeft + static_cast<int16_t>(i) * kLegendCellWidth;
                const auto paletteBucket = i == 0 ? 0 : kBucketCount - i;
                drawingCtx.fillRect(left, kLegendTop, left + kLegendCellWidth - 2, kLegendTop + 7, CargoFlowOverlay::getSaturationColour(paletteBucket), Gfx::RectFlags::none);
            }
            constexpr auto kLegendLabelY = kLegendTop + 13;
            tr.drawStringLeft({ kLegendLeft, kLegendLabelY }, colour, StringIds::low);
            tr.drawStringCentred({ kLegendLeft + kLegendCellWidth * (kBucketCount / 2), kLegendLabelY }, colour, StringIds::medium);
            tr.drawStringRight({ kLegendLeft + kLegendCellWidth * kBucketCount - 1, kLegendLabelY }, colour, StringIds::high);
        }

        constexpr WindowEventList kEvents = {
            .onClose = onClose,
            .onMouseUp = onMouseUp,
            .onUpdate = onUpdate,
            .draw = draw,
        };
    }

    std::optional<Speed16> getTrackSpeed(const World::Pos2& pos, const World::TrackElement& track)
    {
        if (track.trackId() >= World::TrackData::kTrackPieceCount)
        {
            return std::nullopt;
        }
        const auto pieces = World::TrackData::getTrackPiece(track.trackId());
        if (track.sequenceIndex() >= pieces.size())
        {
            return std::nullopt;
        }

        const auto& piece = pieces[track.sequenceIndex()];
        const auto offset = Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, track.rotation());
        const auto start = World::Pos3{ pos, track.baseHeight() } - World::Pos3{ offset, piece.z };
        const auto tad = static_cast<uint16_t>((track.trackId() << 3) | track.rotation());
        const Vehicles::RailTraffic::Edge forward{ start.x, start.y, start.z, tad, track.trackObjectId() };

        const auto& trackSize = World::TrackData::getUnkTrack(tad);
        auto reverseStart = start + trackSize.pos;
        if (trackSize.rotationEnd < 12)
        {
            reverseStart -= World::Pos3{ World::kRotationOffset[trackSize.rotationEnd], 0 };
        }
        const Vehicles::RailTraffic::Edge reverse{ reverseStart.x, reverseStart.y, reverseStart.z, static_cast<uint16_t>(tad | (1U << 2)), track.trackObjectId() };

        auto speed = Vehicles::RailTraffic::getAverageSpeed(forward);
        const auto reverseSpeed = Vehicles::RailTraffic::getAverageSpeed(reverse);
        if (reverseSpeed.has_value() && (!speed.has_value() || *reverseSpeed < *speed))
        {
            speed = reverseSpeed;
        }
        return speed;
    }

    SpeedThresholds calculateSpeedPercentileThresholds(const std::span<const Speed16> speeds)
    {
        std::vector<uint64_t> values;
        values.reserve(speeds.size());
        for (const auto speed : speeds)
        {
            values.push_back(getPercentileValue(speed));
        }
        std::sort(values.begin(), values.end());
        values.erase(std::remove(values.begin(), values.end(), 1), values.end());
        SpeedThresholds thresholds{};
        if (values.empty())
        {
            return thresholds;
        }
        for (size_t i = 1; i < kBucketCount; ++i)
        {
            const auto index = (i * values.size() + kBucketCount - 1) / kBucketCount;
            thresholds[i - 1] = values[std::min(index, values.size() - 1)];
        }
        return thresholds;
    }

    uint8_t getSpeedBucket(const Speed16 speed)
    {
        return getSpeedBucket(speed, _speedThresholds);
    }

    uint8_t getSpeedBucket(const Speed16 speed, const SpeedThresholds& thresholds)
    {
        const auto value = getPercentileValue(speed);
        return static_cast<uint8_t>(std::upper_bound(thresholds.begin(), thresholds.end(), value) - thresholds.begin() + 1);
    }

    PaletteIndex_t getBucketColour(const uint8_t bucket)
    {
        return bucket == 0 ? PaletteIndex::black5 : CargoFlowOverlay::getSaturationColour(bucket == 1 ? 0 : kBucketCount - bucket + 1);
    }

    bool setTooltip(const Viewport& viewport, const Point cursor)
    {
        const auto* main = WindowManager::getMainWindow();
        if (!isOpen() || main == nullptr || main->viewports[0] != &viewport || viewport.hasFlags(ViewportFlags::seeThroughTracks))
        {
            return false;
        }

        const auto [interaction, hitViewport] = ViewportInteraction::getMapCoordinatesFromPos(cursor.x, cursor.y, ~ViewportInteraction::InteractionItemFlags::track);
        if (hitViewport != &viewport || interaction.type != ViewportInteraction::InteractionItem::track)
        {
            return false;
        }
        const auto* track = static_cast<const World::TileElementEntry*>(interaction.object)->as<World::TrackElement>();
        if (track == nullptr)
        {
            return false;
        }
        const auto speed = getTrackSpeed(interaction.pos, *track);
        if (!speed.has_value())
        {
            return false;
        }

        auto args = FormatArguments::mapToolTip(StringIds::rail_speed_overlay_tooltip);
        args.push(*speed);
        args.push<int32_t>(getSpeedBucket(*speed));
        args.push<int32_t>(kBucketCount);
        return true;
    }

    bool isOpen()
    {
        return WindowManager::find(WindowType::railSpeedOverlay) != nullptr;
    }

    Window* open()
    {
        if (auto* window = WindowManager::bringToFront(WindowType::railSpeedOverlay); window != nullptr)
        {
            invalidateMainViewport();
            return window;
        }

        auto* window = WindowManager::createWindowCentred(WindowType::railSpeedOverlay, kWindowSize, WindowFlags::none, kEvents);
        window->setWidgets(kWidgets);
        window->setColour(WindowColour::primary, ObjectManager::get<InterfaceSkinObject>()->windowTitlebarColour);
        window->setColour(WindowColour::secondary, ObjectManager::get<InterfaceSkinObject>()->windowOptionsColour);
        window->initScrollWidgets();
        _historyRevision = Vehicles::RailTraffic::getHistoryRevision();
        refreshSpeedThresholds();
        _refreshTicks = 0;
        invalidateMainViewport();
        return window;
    }

    void toggle()
    {
        if (auto* window = WindowManager::find(WindowType::railSpeedOverlay); window != nullptr)
        {
            WindowManager::close(window);
        }
        else
        {
            open();
        }
    }
}
