// SPDX-License-Identifier: MIT
#include "Entities/EntityManager.h"
#include "GameCommands/GameCommands.h"
#include "GameState.h"
#include "S5/S5.h"
#include "S5/S5File.h"
#include "S5/S5GameState.h"
#include "S5/S5Options.h"
#include "Vehicles/OrderManager.h"
#include "Vehicles/Orders.h"
#include "Vehicles/SharedOrderManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleBogie.h"
#include "Vehicles/VehicleHead.h"
#include "Vehicles/VehicleManager.h"
#include "Vehicles/VehicleReplacement.h"
#include <OpenLoco/Core/FileStream.h>
#include <algorithm>
#include <filesystem>
#include <gtest/gtest.h>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace OpenLoco;
using namespace OpenLoco::Vehicles;

namespace
{
    constexpr const char* kFixtureName = "sus_cargodist.SV5";

    std::optional<fs::path> findFixture()
    {
        const std::vector<std::string> candidates = {
#ifdef OPENLOCO_PROJECT_PATH
            std::string(OPENLOCO_PROJECT_PATH) + "/save_fixtures/" + kFixtureName,
#endif
            std::string("save_fixtures/") + kFixtureName,
            std::string("../../save_fixtures/") + kFixtureName,
            std::string("../../../save_fixtures/") + kFixtureName,
        };
        for (const auto& candidate : candidates)
        {
            if (std::filesystem::exists(candidate))
            {
                return fs::path(candidate);
            }
        }
        return std::nullopt;
    }

    std::string describeOrder(const Order& order)
    {
        switch (order.getType())
        {
            case OrderType::StopAt:
            {
                const auto* stop = order.as<OrderStopAt>();
                return "StopAt(station=" + std::to_string(enumValue(stop->getStation())) + (stop->isUnbunching() ? ", unbunching)" : ")");
            }
            case OrderType::RouteThrough:
            {
                const auto* route = order.as<OrderRouteThrough>();
                return "RouteThrough(station=" + std::to_string(enumValue(route->getStation())) + ")";
            }
            case OrderType::RouteWaypoint:
                return "RouteWaypoint";
            case OrderType::UnloadAll:
                return "UnloadAll(cargo=" + std::to_string(order.as<OrderUnloadAll>()->getCargo()) + ")";
            case OrderType::WaitFor:
                return "WaitFor(cargo=" + std::to_string(order.as<OrderWaitFor>()->getCargo()) + ")";
            case OrderType::End:
                return "End";
        }
        return "Unknown(" + std::to_string(static_cast<uint8_t>(order.getType())) + ")";
    }

    void printTrain(const VehicleHead& head)
    {
        std::cout << "  entity=" << enumValue(head.id)
                  << " ordinal=" << head.ordinalNumber
                  << " owner=" << enumValue(head.owner)
                  << " type=" << static_cast<int>(head.vehicleType)
                  << " mode=" << static_cast<int>(head.mode)
                  << " status=" << static_cast<int>(head.status)
                  << " stationId=" << enumValue(head.stationId)
                  << " currentOrder=" << head.currentOrder
                  << " sizeOfOrderTable=" << head.sizeOfOrderTable
                  << " trackType=" << static_cast<int>(head.getTrackType())
                  << " placed=" << head.isPlaced();
        std::cout << "\n  consist objectIds: ";
        Vehicle train(head);
        for (const auto& car : train.cars)
        {
            std::cout << car.front->objectId << " ";
        }
        std::cout << "\n  orders:\n";
        for (const auto& order : OrderRingView(head.orderTableOffset, head.currentOrder))
        {
            std::cout << "    " << describeOrder(order) << "\n";
        }
    }

    std::unique_ptr<S5::S5File> loadFixture()
    {
        const auto path = findFixture();
        if (!path.has_value())
        {
            return nullptr;
        }
        FileStream stream(*path, StreamMode::read);
        return S5::loadSave(stream);
    }

    void importFixtureState(const S5::S5File& file)
    {
        getGameState() = *S5::importGameState(file.gameState);
        ASSERT_TRUE(SharedOrderManager::restoreState(*file.sharedOrderState));
    }

    class VehicleReplacementFixtureTest : public ::testing::Test
    {
    protected:
        GameState _oldGameState{};

        void SetUp() override
        {
            _oldGameState = getGameState();
            VehicleReplacement::reset();
            SharedOrderManager::reset();
            OrderManager::reset();
            EntityManager::reset();
            GameCommands::setUpdatingCompanyId(CompanyId(0));
        }

        void TearDown() override
        {
            VehicleReplacement::reset();
            SharedOrderManager::reset();
            OrderManager::reset();
            EntityManager::reset();
            GameCommands::setUpdatingCompanyId(CompanyId::neutral);
            getGameState() = _oldGameState;
        }

        // Loads the fixture into the live game state. Skips the test if the
        // fixture or its shared order state is unavailable.
        void loadFixtureOrSkip()
        {
            auto file = loadFixture();
            if (file == nullptr || !file->sharedOrderState.has_value())
            {
                GTEST_SKIP() << "Fixture '" << kFixtureName << "' with shared order state not found";
                return;
            }
            importFixtureState(*file);
        }

        VehicleHead* findHeadByOrdinal(const int16_t ordinal)
        {
            for (auto* head : VehicleManager::VehicleList())
            {
                if (head->ordinalNumber == ordinal)
                {
                    return head;
                }
            }
            return nullptr;
        }
    };
}

TEST_F(VehicleReplacementFixtureTest, LoadsSaveAndFindsTrains131And125)
{
    loadFixtureOrSkip();

    const auto* train131 = findHeadByOrdinal(131);
    const auto* train125 = findHeadByOrdinal(125);
    ASSERT_NE(train131, nullptr) << "Train 131 not found in fixture";
    ASSERT_NE(train125, nullptr) << "Train 125 not found in fixture";

    std::cout << "All train heads in fixture:\n";
    for (auto* head : VehicleManager::VehicleList())
    {
        if (head->vehicleType == VehicleType::train)
        {
            std::cout << "  entity=" << enumValue(head->id) << " ordinal=" << head->ordinalNumber << "\n";
        }
    }

    std::cout << "Train 131:\n";
    printTrain(*train131);
    std::cout << "Train 125:\n";
    printTrain(*train125);

    EXPECT_EQ(train131->owner, train125->owner);
}

TEST_F(VehicleReplacementFixtureTest, Trains131And125ShareOrders)
{
    loadFixtureOrSkip();

    const auto* train131 = findHeadByOrdinal(131);
    const auto* train125 = findHeadByOrdinal(125);
    ASSERT_NE(train131, nullptr);
    ASSERT_NE(train125, nullptr);

    EXPECT_TRUE(SharedOrderManager::isShared(train131->id));
    EXPECT_TRUE(SharedOrderManager::isShared(train125->id));
    EXPECT_EQ(SharedOrderManager::getGroupId(train131->id), SharedOrderManager::getGroupId(train125->id));

    std::cout << "Group members:\n";
    for (const auto member : SharedOrderManager::getMembers(train131->id))
    {
        auto* memberHead = EntityManager::get<VehicleHead>(member);
        std::cout << "  entity=" << enumValue(member) << " ordinal=" << (memberHead != nullptr ? memberHead->ordinalNumber : -1) << "\n";
    }
}

TEST_F(VehicleReplacementFixtureTest, GroupOrderTablesAreByteIdenticalAndContainStopAtOrders)
{
    loadFixtureOrSkip();

    const auto* train131 = findHeadByOrdinal(131);
    const auto* train125 = findHeadByOrdinal(125);
    ASSERT_NE(train131, nullptr);
    ASSERT_NE(train125, nullptr);

    EXPECT_TRUE(SharedOrderManager::areOrdersEqual(*train131, *train125));

    auto summarizeOrders = [](const VehicleHead& head) {
        std::vector<std::string> result;
        for (const auto& order : OrderRingView(head.orderTableOffset, 0))
        {
            result.push_back(describeOrder(order));
        }
        return result;
    };

    const auto orders131 = summarizeOrders(*train131);
    const auto orders125 = summarizeOrders(*train125);
    ASSERT_EQ(orders131.size(), orders125.size());
    for (size_t i = 0; i < orders131.size(); ++i)
    {
        EXPECT_EQ(orders131[i], orders125[i]);
    }

    const bool hasStopAt = std::ranges::any_of(orders131, [](const auto& order) { return order.rfind("StopAt", 0) == 0; });
    EXPECT_TRUE(hasStopAt);
    std::cout << "Train 131 has StopAt order: " << (hasStopAt ? "yes" : "no") << "\n";
}

TEST_F(VehicleReplacementFixtureTest, SchedulingReplacementFrom131QueuesAllOtherGroupMembers)
{
    loadFixtureOrSkip();

    const auto* train131 = findHeadByOrdinal(131);
    ASSERT_NE(train131, nullptr);

    const auto members = SharedOrderManager::getMembers(train131->id);
    std::cout << "Group members (" << members.size() << "):\n";
    for (const auto member : members)
    {
        auto* memberHead = EntityManager::get<VehicleHead>(member);
        std::cout << "  entity=" << enumValue(member) << " ordinal=" << (memberHead != nullptr ? memberHead->ordinalNumber : -1) << "\n";
    }

    ASSERT_TRUE(VehicleReplacement::schedule(train131->id));

    const auto state = VehicleReplacement::captureState();
    std::cout << "Scheduled " << state.requests.size() << " replacement(s):\n";
    for (const auto& request : state.requests)
    {
        const auto* target = EntityManager::get<VehicleHead>(request.target);
        std::cout << "  target entity=" << enumValue(request.target)
                  << " ordinal=" << (target != nullptr ? target->ordinalNumber : -1)
                  << " -> source entity=" << enumValue(request.source) << "\n";
    }

    ASSERT_EQ(state.requests.size(), members.size() - 1);
    for (const auto& request : state.requests)
    {
        EXPECT_EQ(request.source, train131->id);
        EXPECT_NE(request.target, train131->id);
        EXPECT_NE(std::ranges::find(members, request.target), members.end());
    }
}

TEST_F(VehicleReplacementFixtureTest, SavedReplacementStateIsEmpty)
{
    auto file = loadFixture();
    if (file == nullptr)
    {
        GTEST_SKIP() << "Fixture '" << kFixtureName << "' not found";
        return;
    }

    EXPECT_FALSE(file->vehicleReplacementState.has_value());
}

TEST_F(VehicleReplacementFixtureTest, TickDropsPendingPlacementWithMissingHead)
{
    GameCommands::VehiclePlacementArgs placement;
    placement.pos = World::Pos3(512, 384, 32);
    placement.trackAndDirection = 3;
    placement.trackProgress = 5;
    placement.head = EntityId(0);

    VehicleReplacement::State state;
    state.pendingPlacements.push_back({ placement });
    ASSERT_TRUE(VehicleReplacement::restoreState(state));

    VehicleReplacement::tick();

    EXPECT_TRUE(VehicleReplacement::captureState().pendingPlacements.empty());
}

TEST_F(VehicleReplacementFixtureTest, RestoreStateRejectsInvalidPendingPlacements)
{
    GameCommands::VehiclePlacementArgs placement;
    placement.head = EntityId::null;

    VehicleReplacement::State nullHead;
    nullHead.pendingPlacements.push_back({ placement });
    EXPECT_FALSE(VehicleReplacement::restoreState(nullHead));

    GameCommands::VehiclePlacementArgs other;
    other.pos = World::Pos3(16, 0, 0);
    other.trackAndDirection = 2;
    other.trackProgress = 2;
    other.head = EntityId(1);

    GameCommands::VehiclePlacementArgs duplicate;
    duplicate.head = EntityId(1);

    VehicleReplacement::State duplicateHead;
    duplicateHead.pendingPlacements.push_back({ duplicate });
    duplicateHead.pendingPlacements.push_back({ other });
    EXPECT_FALSE(VehicleReplacement::restoreState(duplicateHead));
}

TEST_F(VehicleReplacementFixtureTest, ValidateStateAcceptsPendingPlacementForRealHead)
{
    loadFixtureOrSkip();

    const auto* train131 = findHeadByOrdinal(131);
    ASSERT_NE(train131, nullptr);

    GameCommands::VehiclePlacementArgs placement;
    placement.head = train131->id;

    VehicleReplacement::State state;
    state.pendingPlacements.push_back({ placement });
    EXPECT_TRUE(VehicleReplacement::validateState(state, getGameState()));
}
