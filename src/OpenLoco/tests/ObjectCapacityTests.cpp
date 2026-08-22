#include "Objects/ObjectManager.h"
#include "S5/S5.h"
#include "S5/S5Company.h"
#include "S5/SaveExtension.h"
#include "World/Company.h"
#include <gtest/gtest.h>

using namespace OpenLoco;

namespace
{
    constexpr size_t kVehicleObjectOffset = 389;
    constexpr size_t kLegacyPostVehicleObjectOffset = kVehicleObjectOffset + S5::Limits::kMaxVehicleObjects;
    constexpr size_t kRuntimePostVehicleObjectOffset = kVehicleObjectOffset + Limits::kMaxVehicleObjects;

    static_assert(S5::Limits::kMaxVehicleObjects == 224);
    static_assert(Limits::kMaxVehicleObjects == 1000);
    static_assert(ObjectManager::getMaxObjects(ObjectType::vehicle) == Limits::kMaxVehicleObjects);
    static_assert(ObjectManager::kMaxObjects == 1635);
}

TEST(ObjectCapacityTest, PreservesLegacyRequiredObjectLayout)
{
    std::vector<ObjectHeader> runtimeHeaders(ObjectManager::kMaxObjects);
    for (size_t i = 0; i < runtimeHeaders.size(); ++i)
    {
        runtimeHeaders[i].checksum = static_cast<uint32_t>(i);
    }

    const auto legacyHeaders = S5::exportRequiredObjectHeaders(runtimeHeaders);

    EXPECT_EQ(legacyHeaders.size(), S5::Limits::kMaxObjectHeaders);
    EXPECT_EQ(legacyHeaders[kLegacyPostVehicleObjectOffset - 1].checksum, kLegacyPostVehicleObjectOffset - 1);
    EXPECT_EQ(legacyHeaders[kLegacyPostVehicleObjectOffset].checksum, kRuntimePostVehicleObjectOffset);
    EXPECT_EQ(legacyHeaders.back().checksum, runtimeHeaders.back().checksum);
}

TEST(ObjectCapacityTest, LegacyRequiredObjectsLeaveRuntimeOverflowEmpty)
{
    S5::RequiredObjectHeaders legacyHeaders{};
    for (size_t i = 0; i < legacyHeaders.size(); ++i)
    {
        legacyHeaders[i].checksum = static_cast<uint32_t>(i);
    }

    const auto runtimeHeaders = S5::importRequiredObjectHeaders(legacyHeaders);

    ASSERT_EQ(runtimeHeaders.size(), ObjectManager::kMaxObjects);
    EXPECT_EQ(runtimeHeaders[kLegacyPostVehicleObjectOffset - 1].checksum, kLegacyPostVehicleObjectOffset - 1);
    for (size_t i = kLegacyPostVehicleObjectOffset; i < kRuntimePostVehicleObjectOffset; ++i)
    {
        EXPECT_TRUE(runtimeHeaders[i].isEmpty());
    }
    EXPECT_EQ(runtimeHeaders[kRuntimePostVehicleObjectOffset].checksum, kLegacyPostVehicleObjectOffset);
    EXPECT_EQ(runtimeHeaders.back().checksum, legacyHeaders.back().checksum);
}

TEST(ObjectCapacityTest, ExtendedVehicleHeadersAreInjectedAtExplicitSlots)
{
    S5::RequiredObjectHeaders legacyHeaders{};
    S5::SaveExtension::VehicleObjectState vehicleObjects;
    ObjectHeader first{};
    first.flags = enumValue(ObjectType::vehicle);
    first.checksum = 225;
    ObjectHeader last{};
    last.flags = enumValue(ObjectType::vehicle);
    last.checksum = 999;
    vehicleObjects.objects = { { 225, first }, { 999, last } };

    const auto runtimeHeaders = S5::importRequiredObjectHeaders(legacyHeaders, &vehicleObjects);

    EXPECT_EQ(runtimeHeaders[kVehicleObjectOffset + 225].checksum, 225);
    EXPECT_EQ(runtimeHeaders[kVehicleObjectOffset + 999].checksum, 999);
    EXPECT_TRUE(runtimeHeaders[kVehicleObjectOffset + 224].isEmpty());
}

TEST(ObjectCapacityTest, LegacyCompanyRoundTripOnlyPreservesLegacyUnlocks)
{
    Company company{};
    company.unlockedVehicles.set(0, true);
    company.unlockedVehicles.set(S5::Limits::kMaxVehicleObjects - 1, true);
    company.unlockedVehicles.set(S5::Limits::kMaxVehicleObjects, true);
    company.unlockedVehicles.set(Limits::kMaxVehicleObjects - 1, true);

    const auto legacyCompany = S5::exportCompany(company);
    const auto importedCompany = S5::importCompany(legacyCompany);

    EXPECT_TRUE(importedCompany.isVehicleIndexUnlocked(0));
    EXPECT_TRUE(importedCompany.isVehicleIndexUnlocked(S5::Limits::kMaxVehicleObjects - 1));
    EXPECT_FALSE(importedCompany.isVehicleIndexUnlocked(S5::Limits::kMaxVehicleObjects));
    EXPECT_FALSE(importedCompany.isVehicleIndexUnlocked(Limits::kMaxVehicleObjects - 1));
}
