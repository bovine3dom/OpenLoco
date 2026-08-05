#include "Entities/EntityTweener.h"
#include "Entities/Entity.h"
#include "Map/Tile.h"
#include "OpenLoco.h"
#include "Ui/WindowManager.h"
#include "Vehicles/Vehicle.h"
#include "ViewportManager.h"
#include <algorithm>
#include <cmath>

namespace OpenLoco
{
    using EntityListType = EntityManager::EntityListType;
    using EntityListIterator = EntityManager::ListIterator<EntityBase, &EntityBase::nextEntityId>;

    static constexpr int64_t kTweenPrecision = 1LL << 16;

    static constexpr int32_t roundDivNearest(const int64_t numerator, const int64_t denominator)
    {
        return static_cast<int32_t>(numerator >= 0 ? (numerator + denominator / 2) / denominator : -((-numerator + denominator / 2) / denominator));
    }

    static Ui::Point getInterpolatedRasterPosition(
        const World::Pos3& posA,
        const World::Pos3& posB,
        const uint32_t fraction,
        const uint8_t rotation,
        const ZoomLevel zoom)
    {
        // Preserve legacy tick endpoints while retaining their fractional in-between position.
        const auto vpPosA = World::gameToScreen(posA, rotation);
        const auto vpPosB = World::gameToScreen(posB, rotation);
        const auto rasterScale = zoom.applyInversedTo(1);
        const auto interpolate = [fraction, rasterScale](const int32_t a, const int32_t b) {
            const auto value = static_cast<int64_t>(a) * kTweenPrecision + static_cast<int64_t>(b - a) * fraction;
            return roundDivNearest(value * rasterScale, kTweenPrecision);
        };
        return { interpolate(vpPosA.x, vpPosB.x), interpolate(vpPosA.y, vpPosB.y) };
    }

    static bool hasRasterMovement(
        const World::Pos3& posA,
        const World::Pos3& posB,
        const uint32_t fractionA,
        const uint32_t fractionB,
        const uint8_t rotation)
    {
        return fractionA != fractionB
            && getInterpolatedRasterPosition(posA, posB, fractionA, rotation, ZoomLevel::sixteenfold)
            != getInterpolatedRasterPosition(posA, posB, fractionB, rotation, ZoomLevel::sixteenfold);
    }

    static bool isVisualVehicle(const Vehicles::VehicleBase* vehicle)
    {
        return vehicle != nullptr && (vehicle->isVehicleBody() || vehicle->isVehicleBogie());
    }

    template<EntityListType id, typename Pred>
    void PopulateEntities(std::vector<EntityBase*>& list, std::vector<World::Pos3>& posList, const Pred& pred)
    {
        auto entsView = EntityManager::EntityList<EntityListIterator, id>();
        for (auto* ent : entsView)
        {
            if (!pred(ent))
            {
                continue;
            }

            list.push_back(ent);
            posList.emplace_back(ent->position);
        }
    }

    static EntityTweener _tweener;

    EntityTweener& EntityTweener::get()
    {
        return _tweener;
    }

    void EntityTweener::preTick()
    {
        restore();
        reset();
        PopulateEntities<EntityListType::misc>(_entities, _prePos, [](auto*) { return true; });
        PopulateEntities<EntityListType::vehicle>(_entities, _prePos, [](auto* ent) {
            const auto* vehicle = ent->template asBase<Vehicles::VehicleBase>();
            if (vehicle == nullptr)
            {
                // This can be never null but makes the compiler happy.
                return false;
            }
            return vehicle->isVehicle2() || vehicle->isVehicleBody() || vehicle->isVehicleBogie();
        });

        for (size_t i = 0; i < _entities.size(); ++i)
        {
            const auto id = static_cast<size_t>(_entities[i]->id);
            if (id < _entityIndices.size())
            {
                _entityIndices[id] = static_cast<uint16_t>(i + 1);
            }
        }
    }

    void EntityTweener::postTick()
    {
        for (auto* ent : _entities)
        {
            if (ent == nullptr || ent->id == EntityId::null)
            {
                // Sprite was removed, add a dummy position to keep the index aligned.
                _postPos.emplace_back(0, 0, 0);
            }
            else
            {
                _postPos.emplace_back(ent->position);
            }
        }
    }

    void EntityTweener::removeEntity(const EntityBase* entity)
    {
        const auto id = static_cast<size_t>(entity->id);
        if (id >= _entityIndices.size() || _entityIndices[id] == 0)
        {
            return;
        }

        const auto index = static_cast<size_t>(_entityIndices[id] - 1);
        if (index < _entities.size() && _entities[index] == entity)
        {
            _entities[index] = nullptr;
        }
        _entityIndices[id] = 0;
    }

    void EntityTweener::tween(float alpha)
    {
        const auto previousFraction = _tweenFraction;
        _tweenFraction = static_cast<uint32_t>(std::clamp<int64_t>(std::lround(alpha * kTweenPrecision), 0, kTweenPrecision));
        const float inv = (1.0f - alpha);
        const auto rotation = Ui::WindowManager::getCurrentRotation();

        for (size_t i = 0; i < _entities.size(); ++i)
        {
            auto* ent = _entities[i];
            if (ent == nullptr)
            {
                continue;
            }

            auto& posA = _prePos[i];
            auto& posB = _postPos[i];

            const auto* vehicle = ent->asBase<Vehicles::VehicleBase>();
            const auto rasterPositionChanged = isVisualVehicle(vehicle)
                && hasRasterMovement(posA, posB, previousFraction, _tweenFraction, rotation);
            if (rasterPositionChanged)
            {
                Ui::ViewportManager::invalidate(ent, ZoomLevel::doubled, kRenderPadding);
            }

            if (posA == posB)
            {
                continue;
            }

            if (vehicle != nullptr && vehicle->isVehicle2())
            {
                // The controller remains authoritative; its residual drives followed viewports.
                continue;
            }

            auto newPos = World::Pos3{ static_cast<int16_t>(std::round(posB.x * alpha + posA.x * inv)),
                                       static_cast<int16_t>(std::round(posB.y * alpha + posA.y * inv)),
                                       static_cast<int16_t>(std::round(posB.z * alpha + posA.z * inv)) };

            if (ent->position == newPos)
            {
                continue;
            }

            ent->moveTo(newPos);
            if (rasterPositionChanged)
            {
                Ui::ViewportManager::invalidate(ent, ZoomLevel::doubled, kRenderPadding);
            }
        }
    }

    Ui::Point EntityTweener::getInterpolatedRasterOffset(const EntityBase& entity, const uint8_t rotation, const ZoomLevel zoom) const
    {
        if (zoom >= ZoomLevel::full || entity.baseType != EntityBaseType::vehicle)
        {
            return {};
        }

        const auto id = static_cast<size_t>(entity.id);
        if (id >= _entityIndices.size() || _entityIndices[id] == 0)
        {
            return {};
        }

        const auto index = static_cast<size_t>(_entityIndices[id] - 1);
        if (index >= _entities.size() || index >= _postPos.size() || _entities[index] != &entity)
        {
            return {};
        }

        const auto interpolatedPosition = getInterpolatedRasterPosition(_prePos[index], _postPos[index], _tweenFraction, rotation, zoom);
        const auto integerPosition = World::gameToScreen(entity.position, rotation);
        const auto rasterScale = zoom.applyInversedTo(1);
        const Ui::Point offset{
            interpolatedPosition.x - integerPosition.x * rasterScale,
            interpolatedPosition.y - integerPosition.y * rasterScale,
        };

        const auto* vehicle = entity.asBase<Vehicles::VehicleBase>();
        const auto maxOffset = kRenderPadding * rasterScale;
        if (vehicle != nullptr && !vehicle->isVehicle2()
            && (std::abs(offset.x) > maxOffset || std::abs(offset.y) > maxOffset))
        {
            return {};
        }
        return offset;
    }

    void EntityTweener::restore()
    {
        const auto previousFraction = _tweenFraction;
        _tweenFraction = kTweenPrecision;
        const auto rotation = Ui::WindowManager::getCurrentRotation();

        for (size_t i = 0; i < _entities.size(); ++i)
        {
            auto* ent = _entities[i];
            if (ent == nullptr)
            {
                continue;
            }

            auto& newPos = _postPos[i];
            const auto* vehicle = ent->asBase<Vehicles::VehicleBase>();
            const auto rasterPositionChanged = isVisualVehicle(vehicle)
                && hasRasterMovement(_prePos[i], newPos, previousFraction, _tweenFraction, rotation);
            if (rasterPositionChanged)
            {
                Ui::ViewportManager::invalidate(ent, ZoomLevel::doubled, kRenderPadding);
            }

            if (ent->position == newPos)
            {
                continue;
            }

            ent->moveTo(newPos);
            if (rasterPositionChanged)
            {
                Ui::ViewportManager::invalidate(ent, ZoomLevel::doubled, kRenderPadding);
            }
        }
    }

    void EntityTweener::reset()
    {
        _entities.clear();
        _prePos.clear();
        _postPos.clear();
        _entityIndices.fill(0);
        _tweenFraction = 0;
    }

}
