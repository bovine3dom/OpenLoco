// SPDX-License-Identifier: MIT
#include "../src/Objects/ObjectIndex.cpp"

#include <gtest/gtest.h>
#include <string_view>

using namespace OpenLoco;

namespace
{
    ObjectHeader makeHeader(ObjectType type, std::string_view name, uint32_t checksum)
    {
        ObjectHeader header{};
        header.flags = enumValue(type);
        std::ranges::copy(name, header.name);
        header.checksum = checksum;
        return header;
    }

    ObjectManager::ObjectIndexEntry makeVehicleEntry(ObjectHeader header, VehicleType type, uint32_t numImages = 0)
    {
        ObjectManager::ObjectIndexEntry entry{};
        entry._header = header;
        entry._displayData.numImages = numImages;
        entry._displayData.vehicleSubType = enumValue(type);
        return entry;
    }

    ObjectManager::ObjectIndexEntry makeEntry(ObjectHeader header, uint32_t numImages = 0)
    {
        ObjectManager::ObjectIndexEntry entry{};
        entry._header = header;
        entry._displayData.numImages = numImages;
        return entry;
    }

    bool isSelected(const ObjectManager::ObjectIndexSelection& selection, size_t index)
    {
        return (selection.objectFlags[index] & ObjectManager::SelectedObjectsFlags::selected) != ObjectManager::SelectedObjectsFlags::none;
    }

    void expectSelectionEquals(const ObjectManager::ObjectIndexSelection& actual, const ObjectManager::ObjectIndexSelection& expected)
    {
        EXPECT_EQ(actual.selectionMetaData.numSelectedObjects, expected.selectionMetaData.numSelectedObjects);
        EXPECT_EQ(actual.selectionMetaData.numImages, expected.selectionMetaData.numImages);
        EXPECT_EQ(actual.objectFlags, expected.objectFlags);
    }

    class ObjectIndexSelectionTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            GameRules::reset();
            ObjectManager::_installedObjectList.clear();
        }

        void TearDown() override
        {
            ObjectManager::_installedObjectList.clear();
            GameRules::reset();
        }

        ObjectManager::ObjectIndexSelection makeSelection() const
        {
            ObjectManager::ObjectIndexSelection selection{};
            selection.objectFlags.resize(ObjectManager::_installedObjectList.size());
            return selection;
        }
    };
}

TEST_F(ObjectIndexSelectionTest, SelectsRequestedVehicleCategory)
{
    const auto trainA = makeHeader(ObjectType::vehicle, "TRAIN001", 1);
    const auto bus = makeHeader(ObjectType::vehicle, "BUS00001", 2);
    const auto trainB = makeHeader(ObjectType::vehicle, "TRAIN002", 3);
    const auto road = makeHeader(ObjectType::road, "ROAD0001", 4);
    ObjectManager::_installedObjectList = {
        makeVehicleEntry(trainA, VehicleType::train, 2),
        makeVehicleEntry(bus, VehicleType::bus, 3),
        makeVehicleEntry(trainB, VehicleType::train, 5),
        makeEntry(road, 7),
    };
    ObjectManager::_installedObjectList.back()._displayData.vehicleSubType = enumValue(VehicleType::train);
    auto selection = makeSelection();

    ASSERT_TRUE(selection.selectVehicleObjects(ObjectManager::SelectObjectModes::defaultSelect, VehicleType::train));

    EXPECT_TRUE(isSelected(selection, 0));
    EXPECT_FALSE(isSelected(selection, 1));
    EXPECT_TRUE(isSelected(selection, 2));
    EXPECT_FALSE(isSelected(selection, 3));
    EXPECT_EQ(selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::vehicle)], 2);
    EXPECT_EQ(selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::road)], 0);
    EXPECT_EQ(selection.selectionMetaData.numImages, 7);
}

TEST_F(ObjectIndexSelectionTest, AlreadySelectedCategoryIsANoOp)
{
    const auto train = makeHeader(ObjectType::vehicle, "TRAIN001", 1);
    ObjectManager::_installedObjectList = { makeVehicleEntry(train, VehicleType::train, 2) };
    auto selection = makeSelection();
    selection.objectFlags[0] = ObjectManager::SelectedObjectsFlags::selected | ObjectManager::SelectedObjectsFlags::inUse;
    selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::vehicle)] = 1;
    selection.selectionMetaData.numImages = 2;
    const auto before = selection;

    EXPECT_TRUE(selection.selectVehicleObjects(ObjectManager::SelectObjectModes::defaultSelect, VehicleType::train));
    expectSelectionEquals(selection, before);
}

TEST_F(ObjectIndexSelectionTest, EmptyCategoryIsANoOp)
{
    const auto bus = makeHeader(ObjectType::vehicle, "BUS00001", 1);
    ObjectManager::_installedObjectList = { makeVehicleEntry(bus, VehicleType::bus, 2) };
    auto selection = makeSelection();
    selection.objectFlags[0] = ObjectManager::SelectedObjectsFlags::selected;
    selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::vehicle)] = 1;
    selection.selectionMetaData.numImages = 2;
    const auto before = selection;

    EXPECT_TRUE(selection.selectVehicleObjects(ObjectManager::SelectObjectModes::defaultSelect, VehicleType::ship));
    expectSelectionEquals(selection, before);
}

TEST_F(ObjectIndexSelectionTest, RollsBackWhenALateDependencyCannotBeSelected)
{
    const auto trainA = makeHeader(ObjectType::vehicle, "TRAIN001", 1);
    const auto trainB = makeHeader(ObjectType::vehicle, "TRAIN002", 2);
    const auto bus = makeHeader(ObjectType::vehicle, "BUS00001", 3);
    const auto road = makeHeader(ObjectType::road, "ROAD0001", 4);
    auto trainBEntry = makeVehicleEntry(trainB, VehicleType::train, 3);
    trainBEntry._requiredObjects.push_back(road);
    ObjectManager::_installedObjectList = {
        makeVehicleEntry(trainA, VehicleType::train, 2),
        std::move(trainBEntry),
        makeVehicleEntry(bus, VehicleType::bus, 5),
        makeEntry(road, 7),
    };
    auto selection = makeSelection();
    selection.objectFlags[2] = ObjectManager::SelectedObjectsFlags::selected | ObjectManager::SelectedObjectsFlags::inUse;
    selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::vehicle)] = 1;
    selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::road)] = ObjectManager::getMaxObjects(ObjectType::road);
    selection.selectionMetaData.numImages = 5;
    const auto before = selection;

    EXPECT_FALSE(selection.selectVehicleObjects(ObjectManager::SelectObjectModes::defaultSelect, VehicleType::train));
    expectSelectionEquals(selection, before);
}

TEST_F(ObjectIndexSelectionTest, PreservesOtherVehicleCategories)
{
    const auto train = makeHeader(ObjectType::vehicle, "TRAIN001", 1);
    const auto bus = makeHeader(ObjectType::vehicle, "BUS00001", 2);
    const auto aircraft = makeHeader(ObjectType::vehicle, "AIR00001", 3);
    ObjectManager::_installedObjectList = {
        makeVehicleEntry(train, VehicleType::train, 2),
        makeVehicleEntry(bus, VehicleType::bus, 3),
        makeVehicleEntry(aircraft, VehicleType::aircraft, 5),
    };
    auto selection = makeSelection();
    selection.objectFlags[1] = ObjectManager::SelectedObjectsFlags::selected | ObjectManager::SelectedObjectsFlags::inUse;
    selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::vehicle)] = 1;
    selection.selectionMetaData.numImages = 3;

    ASSERT_TRUE(selection.selectVehicleObjects(ObjectManager::SelectObjectModes::defaultSelect, VehicleType::train));

    EXPECT_TRUE(isSelected(selection, 0));
    EXPECT_EQ(selection.objectFlags[1], ObjectManager::SelectedObjectsFlags::selected | ObjectManager::SelectedObjectsFlags::inUse);
    EXPECT_FALSE(isSelected(selection, 2));
    EXPECT_EQ(selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::vehicle)], 2);
    EXPECT_EQ(selection.selectionMetaData.numImages, 5);
}

TEST_F(ObjectIndexSelectionTest, LegacyRuleRejectsVehicleObject225)
{
    for (uint32_t i = 0; i < 225; ++i)
    {
        ObjectManager::_installedObjectList.push_back(makeVehicleEntry(makeHeader(ObjectType::vehicle, "VEHICLE ", i), VehicleType::train));
    }
    auto selection = makeSelection();

    for (size_t i = 0; i < S5::Limits::kMaxVehicleObjects; ++i)
    {
        ASSERT_TRUE(selection.selectObject(ObjectManager::SelectObjectModes::defaultSelect, ObjectManager::_installedObjectList[i]._header));
    }
    EXPECT_FALSE(selection.selectObject(ObjectManager::SelectObjectModes::defaultSelect, ObjectManager::_installedObjectList[224]._header));
    EXPECT_EQ(selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::vehicle)], 224);
    EXPECT_EQ(ObjectManager::getMaxSelectableObjects(ObjectType::vehicle), 224);
}

TEST_F(ObjectIndexSelectionTest, ExtendedRuleAllowsVehicleObject225)
{
    GameRules::restoreState({ .extendedVehicleObjects = true });
    for (uint32_t i = 0; i < 225; ++i)
    {
        ObjectManager::_installedObjectList.push_back(makeVehicleEntry(makeHeader(ObjectType::vehicle, "VEHICLE ", i), VehicleType::train));
    }
    auto selection = makeSelection();

    for (const auto& entry : ObjectManager::_installedObjectList)
    {
        ASSERT_TRUE(selection.selectObject(ObjectManager::SelectObjectModes::defaultSelect, entry._header));
    }
    EXPECT_EQ(selection.selectionMetaData.numSelectedObjects[enumValue(ObjectType::vehicle)], 225);
    EXPECT_EQ(ObjectManager::getMaxSelectableObjects(ObjectType::vehicle), Limits::kMaxVehicleObjects);
    EXPECT_TRUE(selection.hasExtendedVehicleObjectsSelected());
}

TEST_F(ObjectIndexSelectionTest, TgvLaPosteReservesAndReleasesDuplicateImages)
{
    ObjectManager::_installedObjectList = {
        makeVehicleEntry(kOfficialTgvPassengerCarriageHeader, VehicleType::train, 10),
        makeEntry(kOfficialMailCargoHeader, 3),
    };
    auto selection = makeSelection();

    ASSERT_TRUE(selection.selectObject(ObjectManager::SelectObjectModes::defaultSelect, kOfficialTgvPassengerCarriageHeader));
    EXPECT_EQ(selection.selectionMetaData.numImages, 10);
    ASSERT_TRUE(selection.selectObject(ObjectManager::SelectObjectModes::defaultSelect, kOfficialMailCargoHeader));
    EXPECT_EQ(selection.selectionMetaData.numImages, 23);

    ASSERT_TRUE(selection.selectObject(ObjectManager::SelectObjectModes::defaultDeselect, kOfficialMailCargoHeader));
    EXPECT_EQ(selection.selectionMetaData.numImages, 10);
}

TEST_F(ObjectIndexSelectionTest, TgvLaPosteRejectsInsufficientDuplicateImageCapacity)
{
    constexpr auto kTgvImages = Gfx::G1ExpectedCount::kObjects / 2 + 1;
    ObjectManager::_installedObjectList = {
        makeVehicleEntry(kOfficialTgvPassengerCarriageHeader, VehicleType::train, kTgvImages),
        makeEntry(kOfficialMailCargoHeader),
    };
    auto selection = makeSelection();

    ASSERT_TRUE(selection.selectObject(ObjectManager::SelectObjectModes::defaultSelect, kOfficialTgvPassengerCarriageHeader));
    EXPECT_FALSE(selection.selectObject(ObjectManager::SelectObjectModes::defaultSelect, kOfficialMailCargoHeader));
    EXPECT_FALSE(isSelected(selection, 1));
    EXPECT_EQ(selection.selectionMetaData.numImages, kTgvImages);
}
