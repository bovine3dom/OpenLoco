#pragma once

#include "EntityManager.h"
#include <OpenLoco/Engine/Limits.h>
#include <OpenLoco/Engine/Ui/Point.hpp>
#include <OpenLoco/Engine/World.hpp>
#include <OpenLoco/ZoomLevel.hpp>
#include <array>
#include <vector>

namespace OpenLoco
{
    class EntityTweener
    {
        std::vector<EntityBase*> _entities;
        std::vector<World::Pos3> _prePos;
        std::vector<World::Pos3> _postPos;
        std::array<uint16_t, Limits::kMaxEntities> _entityIndices{};
        uint32_t _tweenFraction{};

    public:
        static constexpr int32_t kRenderPadding = 2;

        static EntityTweener& get();

        void preTick();
        void postTick();
        void removeEntity(const EntityBase* entity);
        void tween(float alpha);
        Ui::Point getInterpolatedRasterOffset(const EntityBase& entity, uint8_t rotation, ZoomLevel zoom) const;
        void restore();
        void reset();
    };
}
