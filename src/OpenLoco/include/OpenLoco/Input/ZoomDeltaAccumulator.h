#pragma once

#include "ZoomLevel.hpp"
#include <OpenLoco/Engine/Ui/Point.hpp>

namespace OpenLoco::Input
{
    class ZoomDeltaAccumulator
    {
    public:
        Ui::Point apply(const Ui::Point delta, const ZoomLevel zoom)
        {
            if (_zoom != zoom)
            {
                reset();
                _zoom = zoom;
            }

            const auto level = static_cast<int8_t>(zoom);
            if (level >= ZoomLevel::full)
            {
                return { zoom.applyTo(delta.x), zoom.applyTo(delta.y) };
            }

            const auto divisor = int64_t{ 1 } << -level;
            const auto applyAxis = [divisor](const int32_t value, AxisState& state) {
                state.input += value;
                const auto output = state.input / divisor;
                const auto deltaOutput = output - state.output;
                state.output = output;
                return static_cast<int32_t>(deltaOutput);
            };
            return { applyAxis(delta.x, _x), applyAxis(delta.y, _y) };
        }

        void reset()
        {
            _x = {};
            _y = {};
        }

        void resetX()
        {
            _x = {};
        }

        void resetY()
        {
            _y = {};
        }

    private:
        struct AxisState
        {
            int64_t input{};
            int64_t output{};
        };

        ZoomLevel _zoom{ ZoomLevel::full };
        AxisState _x{};
        AxisState _y{};
    };
}
