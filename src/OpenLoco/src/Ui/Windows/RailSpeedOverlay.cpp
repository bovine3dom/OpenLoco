// SPDX-License-Identifier: MIT
#include "Ui/Windows/RailSpeedOverlay.h"

#include "Graphics/DrawingContext.h"
#include "Graphics/ImageIds.h"
#include "Graphics/TextRenderer.h"
#include "Localisation/FormatArguments.hpp"
#include "Localisation/StringIds.h"
#include "Map/Track/TrackData.h"
#include "Map/TrackElement.h"
#include "Objects/InterfaceSkinObject.h"
#include "Objects/ObjectManager.h"
#include "Ui/Widgets/CaptionWidget.h"
#include "Ui/Widgets/FrameWidget.h"
#include "Ui/Widgets/ImageButtonWidget.h"
#include "Ui/Widgets/PanelWidget.h"
#include "Ui/WindowManager.h"
#include "Vehicles/RailTraffic.h"
#include <OpenLoco/Math/Vector.hpp>
#include <algorithm>
#include <array>

using namespace OpenLoco::Literals;

namespace OpenLoco::Ui::Windows::RailSpeedOverlay
{
    namespace
    {
        constexpr Ui::Size kWindowSize = { 252, 73 };
        constexpr int16_t kLegendLeft = 18;
        constexpr int16_t kLegendTop = 45;
        constexpr int16_t kLegendCellWidth = 27;

        constexpr std::array kSpeedThresholds = {
            10_mph,
            20_mph,
            30_mph,
            45_mph,
            60_mph,
            80_mph,
            120_mph,
        };

        constexpr std::array kBucketColours = {
            Colour::red,
            Colour::darkOrange,
            Colour::orange,
            Colour::amber,
            Colour::yellow,
            Colour::mutedAvocadoGreen,
            Colour::mutedGrassGreen,
            Colour::green,
        };

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

        void invalidateMainViewport()
        {
            if (auto* main = WindowManager::getMainWindow(); main != nullptr)
            {
                main->invalidate();
            }
        }

        void drawSpeed(Gfx::TextRenderer& tr, const Point& pos, const Speed16 speed, const bool centred)
        {
            FormatArguments args{};
            args.push(speed);
            if (centred)
            {
                tr.drawStringCentred(pos, Colour::white, StringIds::velocity, args);
            }
            else
            {
                tr.drawStringLeft(pos, Colour::white, StringIds::velocity, args);
            }
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
                invalidateMainViewport();
            }
        }

        void draw(Window& self, Gfx::DrawingContext& drawingCtx)
        {
            self.draw(drawingCtx);
            auto tr = Gfx::TextRenderer(drawingCtx);
            tr.setCurrentFont(Gfx::Font::small);
            tr.drawStringCentred({ kWindowSize.width / 2, 20 }, Colour::white, StringIds::rail_speed_overlay_description);
            tr.drawStringCentred({ kWindowSize.width / 2, 29 }, Colour::white, StringIds::rail_speed_overlay_no_data);
            for (size_t i = 0; i < kBucketColours.size(); ++i)
            {
                const auto left = kLegendLeft + static_cast<int16_t>(i) * kLegendCellWidth;
                drawingCtx.fillRect(left, kLegendTop, left + kLegendCellWidth - 2, kLegendTop + 7, Colours::getShade(kBucketColours[i], 7), Gfx::RectFlags::none);
            }
            constexpr auto kLegendLabelY = kLegendTop + 11;
            drawSpeed(tr, { kLegendLeft, kLegendLabelY }, 0_mph, false);
            drawSpeed(tr, { kLegendLeft + kLegendCellWidth * 3, kLegendLabelY }, 30_mph, true);
            drawSpeed(tr, { kLegendLeft + kLegendCellWidth * 5, kLegendLabelY }, 60_mph, true);
            drawSpeed(tr, { kLegendLeft + kLegendCellWidth * 7, kLegendLabelY }, 120_mph, true);
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

    uint8_t getSpeedBucket(const Speed16 speed)
    {
        return static_cast<uint8_t>(1 + std::ranges::count_if(kSpeedThresholds, [speed](const auto threshold) { return speed >= threshold; }));
    }

    Colour getBucketColour(const uint8_t bucket)
    {
        return bucket == 0 || bucket > kBucketColours.size() ? Colour::black : kBucketColours[bucket - 1];
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
