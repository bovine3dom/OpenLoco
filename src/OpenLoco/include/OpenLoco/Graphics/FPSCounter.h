#pragma once

#include <OpenLoco/Engine/Ui/Rect.hpp>

namespace OpenLoco::Gfx
{
    class DrawingContext;

    Ui::Rect drawFPS(DrawingContext& drawingCtx, bool measure = true);
}
