#include <OpenLoco/Vehicles/SignalFuzzer.h>
#include <array>
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::Vehicles;

TEST(SignalFuzzerTest, CaseGenerationIsDeterministic)
{
    SignalFuzzer::Options options{};
    options.baseSave = "signal_stress.SV5";
    options.focusTown = "Beachtown";
    options.seed = 1234;
    options.ticks = 5000;
    constexpr std::array candidates{ EntityId(10), EntityId(20), EntityId(30) };

    const auto first = SignalFuzzer::makeCase(options, 7, candidates);
    const auto second = SignalFuzzer::makeCase(options, 7, candidates);

    EXPECT_TRUE(first.injectBreakdown);
    EXPECT_EQ(first.targetVehicle, second.targetVehicle);
    EXPECT_EQ(first.earliestBreakdownTick, second.earliestBreakdownTick);
}

TEST(SignalFuzzerTest, BaselineCaseDoesNotInjectBreakdown)
{
    SignalFuzzer::Options options{};
    options.baseSave = "signal_stress.SV5";
    options.ticks = 1000;
    constexpr std::array candidates{ EntityId(10) };

    const auto fuzzCase = SignalFuzzer::makeCase(options, 0, candidates);

    EXPECT_FALSE(fuzzCase.injectBreakdown);
    EXPECT_EQ(fuzzCase.targetVehicle, EntityId::null);
}

TEST(SignalFuzzerTest, FlatAllCyclesConcreteLayouts)
{
    SignalFuzzer::Options options{};
    options.baseSave = "signal_stress.SV5";
    options.layout = SignalFuzzer::Layout::flatAll;

    EXPECT_EQ(SignalFuzzer::makeCase(options, 0, {}).layout, SignalFuzzer::Layout::flatMerge);
    EXPECT_EQ(SignalFuzzer::makeCase(options, 1, {}).layout, SignalFuzzer::Layout::flatFan);
    EXPECT_EQ(SignalFuzzer::makeCase(options, 2, {}).layout, SignalFuzzer::Layout::flatInterchange);
    EXPECT_EQ(SignalFuzzer::makeCase(options, 3, {}).layout, SignalFuzzer::Layout::flatMerge);
}

TEST(SignalFuzzerTest, FlatAllStartsWithOneBaselinePerLayout)
{
    SignalFuzzer::Options options{};
    options.baseSave = "signal_stress.SV5";
    options.layout = SignalFuzzer::Layout::flatAll;
    constexpr std::array candidates{ EntityId(10) };

    EXPECT_FALSE(SignalFuzzer::makeCase(options, 0, candidates).injectBreakdown);
    EXPECT_FALSE(SignalFuzzer::makeCase(options, 1, candidates).injectBreakdown);
    EXPECT_FALSE(SignalFuzzer::makeCase(options, 2, candidates).injectBreakdown);
    EXPECT_TRUE(SignalFuzzer::makeCase(options, 3, candidates).injectBreakdown);
}

TEST(SignalFuzzerTest, LayoutNamesRoundTrip)
{
    constexpr std::array layouts{
        SignalFuzzer::Layout::fixture,
        SignalFuzzer::Layout::flatMerge,
        SignalFuzzer::Layout::flatFan,
        SignalFuzzer::Layout::flatInterchange,
        SignalFuzzer::Layout::flatAll,
    };
    for (const auto layout : layouts)
    {
        EXPECT_EQ(SignalFuzzer::parseLayout(SignalFuzzer::layoutName(layout)), layout);
    }
    EXPECT_FALSE(SignalFuzzer::parseLayout("unknown").has_value());
}

TEST(SignalFuzzerTest, CaseRoundTripsThroughYaml)
{
    SignalFuzzer::Case fuzzCase{};
    fuzzCase.baseSave = "/tmp/signal stress.SV5";
    fuzzCase.focusTown = "Beach Town";
    fuzzCase.layout = SignalFuzzer::Layout::flatFan;
    fuzzCase.seed = 42;
    fuzzCase.caseIndex = 9;
    fuzzCase.ticks = 10000;
    fuzzCase.targetVehicle = EntityId(123);
    fuzzCase.earliestBreakdownTick = 4567;
    fuzzCase.injectBreakdown = true;

    const auto restored = SignalFuzzer::deserialiseCase(SignalFuzzer::serialiseCase(fuzzCase));

    ASSERT_TRUE(restored.has_value());
    EXPECT_EQ(restored->baseSave, fuzzCase.baseSave);
    EXPECT_EQ(restored->focusTown, fuzzCase.focusTown);
    EXPECT_EQ(restored->layout, fuzzCase.layout);
    EXPECT_EQ(restored->seed, fuzzCase.seed);
    EXPECT_EQ(restored->caseIndex, fuzzCase.caseIndex);
    EXPECT_EQ(restored->ticks, fuzzCase.ticks);
    EXPECT_EQ(restored->targetVehicle, fuzzCase.targetVehicle);
    EXPECT_EQ(restored->earliestBreakdownTick, fuzzCase.earliestBreakdownTick);
    EXPECT_EQ(restored->injectBreakdown, fuzzCase.injectBreakdown);
}

TEST(SignalFuzzerTest, VersionOneCaseDefaultsToFixtureLayout)
{
    constexpr auto yaml = R"(
version: 1
base_save: /tmp/signal_stress.SV5
focus_town: Beachtown
seed: 42
case_index: 9
ticks: 10000
target_vehicle: 123
earliest_breakdown_tick: 4567
inject_breakdown: true
)";

    const auto restored = SignalFuzzer::deserialiseCase(yaml);

    ASSERT_TRUE(restored.has_value());
    EXPECT_EQ(restored->layout, SignalFuzzer::Layout::fixture);
}
