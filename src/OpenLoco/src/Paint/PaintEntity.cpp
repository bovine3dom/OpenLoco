#include "Paint/PaintEntity.h"
#include "Config.h"
#include "Effects/Effect.h"
#include "Entities/EntityManager.h"
#include "Entities/EntityTweener.h"
#include "Graphics/RenderTarget.h"
#include "Map/Tile.h"
#include "Paint/Paint.h"
#include "Paint/PaintEffectEntity.h"
#include "Paint/PaintVehicle.h"
#include "Ui/ViewportInteraction.h"
#include "Vehicles/Vehicle.h"

using namespace OpenLoco::Ui::ViewportInteraction;

namespace OpenLoco::Paint
{
    template<typename FilterType>
    static void paintEntitiesWithFilter(PaintSession& session, const World::Pos2& loc, FilterType&& filter)
    {
        if (Config::get().vehiclesMinScale < session.getZoom())
        {
            return;
        }

        if (loc.x >= 0x4000 || loc.y >= 0x4000)
        {
            return;
        }

        EntityManager::EntityTileList entities(loc);
        for (auto* entity : entities)
        {
            // TODO: Create a rect from context dims
            auto left = session.getWorldX();
            auto top = session.getWorldY();
            auto right = left + session.getWorldWidth();
            auto bottom = top + session.getWorldHeight();
            const auto renderPadding = session.getZoom() < ZoomLevel::full ? EntityTweener::kRenderPadding : 0;

            // TODO: Create a rect from sprite dims and use a contains function
            if (entity->spriteTop - renderPadding > bottom)
            {
                continue;
            }
            if (entity->spriteBottom + renderPadding <= top)
            {
                continue;
            }
            if (entity->spriteLeft - renderPadding > right)
            {
                continue;
            }
            if (entity->spriteRight + renderPadding <= left)
            {
                continue;
            }
            if (!filter(entity))
            {
                continue;
            }
            session.setCurrentItem(entity);
            const auto rasterOffset = EntityTweener::get().getInterpolatedRasterOffset(*entity, session.getRotation(), session.getZoom());
            session.setEntityPosition(entity->position, rasterOffset);
            session.setItemType(InteractionItem::entity);
            switch (entity->baseType)
            {
                case EntityBaseType::vehicle:
                    paintVehicleEntity(session, entity->asBase<Vehicles::VehicleBase>());
                    break;
                case EntityBaseType::effect:
                    paintEffectEntity(session, entity->asBase<EffectEntity>());
                    break;
                case EntityBaseType::null:
                    // Nothing to paint
                    break;
            }
        }
    }

    // 0x0046FA88
    void paintEntities(PaintSession& session, const World::Pos2& loc)
    {
        paintEntitiesWithFilter(session, loc, [](const EntityBase*) { return true; });
    }

    static bool isEntityFlyingOrFloating(const EntityBase* entity)
    {
        auto* vehicle = entity->asBase<Vehicles::VehicleBase>();
        if (vehicle == nullptr)
        {
            return false;
        }
        switch (vehicle->getTransportMode())
        {
            case TransportMode::air:
            case TransportMode::water:
                return true;
            case TransportMode::rail:
            case TransportMode::road:
                return false;
        }
        return false;
    }

    // 0x0046FB67
    void paintEntities2(PaintSession& session, const World::Pos2& loc)
    {
        paintEntitiesWithFilter(session, loc, isEntityFlyingOrFloating);
    }
}
