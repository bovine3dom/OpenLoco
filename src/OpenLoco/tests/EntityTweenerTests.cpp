#include "Entities/EntityManager.h"
#include "Entities/EntityTweener.h"
#include "Map/Tile.h"
#include "Ui/WindowManager.h"
#include "Vehicles/Vehicle2.h"
#include "Vehicles/VehicleBody.h"
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::Vehicles;

namespace
{
    class EntityTweenerTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            EntityTweener::get().reset();
            EntityManager::reset();
            Ui::WindowManager::setCurrentRotation(0);
        }

        void TearDown() override
        {
            EntityTweener::get().reset();
            EntityManager::reset();
        }

        template<typename T>
        static T* createComponent(const World::Pos3& position)
        {
            auto* entity = EntityManager::createEntityVehicle();
            if (entity == nullptr)
            {
                return nullptr;
            }

            entity->baseType = EntityBaseType::vehicle;
            auto* component = reinterpret_cast<T*>(entity);
            component->setSubType(T::kVehicleThingType);
            component->moveTo(position);
            return component;
        }
    };
}

TEST_F(EntityTweenerTest, PreservesFractionalMovementAtMagnifiedZoom)
{
    auto* body = createComponent<VehicleBody>({ 64, 64, 0 });
    ASSERT_NE(body, nullptr);

    auto& tweener = EntityTweener::get();
    tweener.preTick();
    body->moveTo({ 68, 64, 0 });
    tweener.postTick();
    tweener.tween(0.25F);

    EXPECT_EQ(body->position, (World::Pos3{ 65, 64, 0 }));
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::doubled), (Ui::Point{ 0, 1 }));
    for (uint8_t rotation = 0; rotation < 4; ++rotation)
    {
        EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, rotation, ZoomLevel::quadrupled), (Ui::Point{ 0, 2 }));
    }
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::sixteenfold), (Ui::Point{ 0, 8 }));
}

TEST_F(EntityTweenerTest, PreservesExactIsometricPhaseAtMagnifiedZoom)
{
    auto* body = createComponent<VehicleBody>({ 65, 64, 0 });
    ASSERT_NE(body, nullptr);

    auto& tweener = EntityTweener::get();
    for (uint8_t rotation = 0; rotation < 4; ++rotation)
    {
        EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, rotation, ZoomLevel::doubled), (Ui::Point{ 0, 1 }));
        EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, rotation, ZoomLevel::quadrupled), (Ui::Point{ 0, 2 }));
        EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, rotation, ZoomLevel::eightfold), (Ui::Point{ 0, 4 }));
        EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, rotation, ZoomLevel::sixteenfold), (Ui::Point{ 0, 8 }));
    }

    tweener.preTick();
    tweener.postTick();
    tweener.tween(0.5F);
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::quadrupled), (Ui::Point{ 0, 2 }));
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::full), Ui::Point{});
}

TEST_F(EntityTweenerTest, RetainsExactPhaseAfterTweenReset)
{
    auto* body = createComponent<VehicleBody>({ 64, 64, 0 });
    ASSERT_NE(body, nullptr);

    auto& tweener = EntityTweener::get();
    tweener.preTick();
    body->moveTo({ 65, 64, 0 });
    tweener.postTick();
    tweener.tween(1.0F);
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::quadrupled), (Ui::Point{ 0, 2 }));

    tweener.reset();
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::quadrupled), (Ui::Point{ 0, 2 }));
}

TEST_F(EntityTweenerTest, ProjectsSuccessiveWorldPositionsUniformly)
{
    auto* body = createComponent<VehicleBody>({ 64, 64, 0 });
    ASSERT_NE(body, nullptr);

    constexpr std::array<Ui::Point, 4> kExpectedSteps = {
        Ui::Point{ -4, 2 },
        Ui::Point{ -4, -2 },
        Ui::Point{ 4, -2 },
        Ui::Point{ 4, 2 },
    };
    auto& tweener = EntityTweener::get();
    for (uint8_t rotation = 0; rotation < 4; ++rotation)
    {
        Ui::Point previousPosition{};
        for (int16_t x = 64; x <= 66; ++x)
        {
            body->moveTo({ x, 64, 0 });
            const auto integerPosition = World::gameToScreen(body->position, rotation);
            const auto offset = tweener.getInterpolatedRasterOffset(*body, rotation, ZoomLevel::quadrupled);
            const Ui::Point rasterPosition{ integerPosition.x * 4 + offset.x, integerPosition.y * 4 + offset.y };
            if (x != 64)
            {
                EXPECT_EQ(rasterPosition - previousPosition, kExpectedSteps[rotation]);
            }
            previousPosition = rasterPosition;
        }
    }
}

TEST_F(EntityTweenerTest, LeavesLegacyZoomLevelsAndIntegralTickEndpointsUnchanged)
{
    auto* body = createComponent<VehicleBody>({ 64, 64, 0 });
    ASSERT_NE(body, nullptr);

    auto& tweener = EntityTweener::get();
    tweener.preTick();
    body->moveTo({ 68, 64, 0 });
    tweener.postTick();

    tweener.tween(0.0F);
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::quadrupled), Ui::Point{});

    tweener.tween(0.25F);
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::full), Ui::Point{});
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::half), Ui::Point{});

    tweener.restore();
    EXPECT_EQ(body->position, (World::Pos3{ 68, 64, 0 }));
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*body, 0, ZoomLevel::quadrupled), Ui::Point{});
}

TEST_F(EntityTweenerTest, TracksVehicleControllerForFollowView)
{
    auto* controller = createComponent<Vehicle2>({ 64, 64, 0 });
    ASSERT_NE(controller, nullptr);

    auto& tweener = EntityTweener::get();
    tweener.preTick();
    controller->moveTo({ 68, 64, 0 });
    tweener.postTick();
    tweener.tween(0.25F);

    EXPECT_EQ(controller->position, (World::Pos3{ 68, 64, 0 }));
    EXPECT_EQ(tweener.getInterpolatedRasterOffset(*controller, 0, ZoomLevel::quadrupled), (Ui::Point{ 12, -6 }));
}
