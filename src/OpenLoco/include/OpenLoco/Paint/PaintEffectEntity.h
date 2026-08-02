#pragma once

#include "Effects/Effect.h"
#include "Paint.h"

namespace OpenLoco::Gfx
{
    class DrawingContext;
}
namespace OpenLoco::Ui
{
    struct Viewport;
}

namespace OpenLoco::Paint
{

    /*
     * @param base->x @<ax>
     * @param base->y @<cx>
     * @param base->z @<dx>
     * @param ((base->sprite_yaw + (session.getRotation() << 4)) & 0x3F) @<ebx>
     * @param base @<esi>
     */
    void paintEffectEntity(PaintSession& session, EffectEntity* base);
    void drawMoneyEffects(Gfx::DrawingContext& drawingCtx, const Ui::Viewport& viewport);
}
