#include "Ui/Windows/ProductionHeatmap.h"

#include <gtest/gtest.h>
#include <limits>
#include <numeric>

using namespace OpenLoco;
using namespace OpenLoco::Ui::Windows::ProductionHeatmap;

TEST(ProductionHeatmap, DistributesPhysicalOutputAcrossUniqueFootprint)
{
    const ProductionSource source{ 100, { { 1, 1 }, { 1, 1 }, { 2, 1 } } };
    const auto layers = buildProductionLayers(std::span{ &source, 1 }, 4, 4, 1);

    EXPECT_EQ(layers.physical.values[5], 50U);
    EXPECT_EQ(layers.physical.values[6], 50U);
    EXPECT_EQ(std::accumulate(layers.physical.values.begin(), layers.physical.values.end(), uint64_t{}), 100U);
}

TEST(ProductionHeatmap, CountsSourceOnceAcrossOverlappingCatchmentRegions)
{
    const ProductionSource source{ 100, { { 1, 1 }, { 2, 1 } } };
    const auto layers = buildProductionLayers(std::span{ &source, 1 }, 5, 4, 1);

    EXPECT_EQ(layers.stationPotential.values[1 * 5 + 1], 100U);
    EXPECT_EQ(layers.stationPotential.values[1 * 5 + 2], 100U);
    EXPECT_EQ(layers.stationPotential.values[1 * 5 + 3], 100U);
    EXPECT_EQ(layers.stationPotential.values[3 * 5 + 1], 0U);
}

TEST(ProductionHeatmap, AddsIndependentSources)
{
    const std::array sources = {
        ProductionSource{ 100, { { 1, 1 } } },
        ProductionSource{ 60, { { 2, 1 } } },
    };
    const auto layers = buildProductionLayers(sources, 4, 4, 1);

    EXPECT_EQ(layers.stationPotential.values[1 * 4 + 1], 160U);
    EXPECT_EQ(layers.stationPotential.values[1 * 4 + 0], 100U);
    EXPECT_EQ(layers.stationPotential.values[1 * 4 + 3], 60U);
}

TEST(ProductionHeatmap, PercentileBucketsIgnoreZeroAndRemainMonotonic)
{
    const std::array<uint64_t, 10> values = { 0, 1, 1, 2, 3, 4, 5, 6, 7, 8 };
    const auto thresholds = calculatePercentileThresholds(values);

    EXPECT_EQ(getPercentileBucket(0, thresholds), 0);
    EXPECT_EQ(getPercentileBucket(1, thresholds), 1);
    for (uint64_t value = 2; value <= 8; ++value)
    {
        EXPECT_LE(getPercentileBucket(value - 1, thresholds), getPercentileBucket(value, thresholds));
    }
    EXPECT_EQ(getPercentileBucket(8, thresholds), kBucketCount);
}

TEST(ProductionHeatmap, SparsePercentilesUseTheFullColourRange)
{
    const std::array<uint64_t, 3> values = { 0, 1, 2 };
    const auto thresholds = calculatePercentileThresholds(values);

    EXPECT_TRUE(std::is_sorted(thresholds.begin(), thresholds.end()));
    EXPECT_EQ(getPercentileBucket(1, thresholds), 1);
    EXPECT_EQ(getPercentileBucket(2, thresholds), kBucketCount);
}

TEST(ProductionHeatmap, NonDrawableBordersDoNotAffectPercentiles)
{
    const ProductionSource source{ 100, { { 0, 0 }, { 3, 3 } } };
    const auto layers = buildProductionLayers(std::span{ &source, 1 }, 5, 5, 1, true);

    EXPECT_EQ(layers.physical.values[0], 0U);
    EXPECT_NE(layers.physical.values[3 * 5 + 3], 0U);
    EXPECT_NE(layers.stationPotential.values[3 * 5 + 3], 0U);
    EXPECT_EQ(layers.stationPotential.values[4 * 5 + 4], 0U);
}

TEST(ProductionHeatmap, SaturatesCombinedProductionWithoutWrapping)
{
    const std::array sources = {
        ProductionSource{ std::numeric_limits<uint64_t>::max(), { { 1, 1 } } },
        ProductionSource{ 1, { { 1, 1 } } },
    };
    const auto layers = buildProductionLayers(sources, 3, 3, 1);

    EXPECT_EQ(layers.physical.values[1 * 3 + 1], std::numeric_limits<uint64_t>::max());
    EXPECT_EQ(layers.stationPotential.values[1 * 3 + 1], std::numeric_limits<uint64_t>::max());
}
