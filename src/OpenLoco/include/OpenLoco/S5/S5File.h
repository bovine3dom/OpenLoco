#pragma once

#include "Objects/Object.h"
#include "S5/S5.h"
#include "S5/S5GameState.h"
#include "S5/S5TileElement.h"
#include <OpenLoco/CargoDist/CargoDist.h>
#include <OpenLoco/Vehicles/RoutingManager.h>
#include <OpenLoco/Vehicles/RailTraffic.h>
#include <OpenLoco/Vehicles/SharedOrderManager.h>
#include <OpenLoco/Vehicles/VehicleAutoRenewal.h>
#include <optional>

namespace OpenLoco::S5
{
    struct S5File
    {
        Header header;
        std::unique_ptr<Options> scenarioOptions;
        std::unique_ptr<SaveDetails> saveDetails;
        ObjectHeader requiredObjects[859];
        GameState gameState;
        std::vector<TileElement> tileElements;
        std::vector<std::pair<ObjectHeader, std::vector<std::byte>>> packedObjects;
        std::optional<CargoDist::State> cargoDistState;
        std::optional<Vehicles::SharedOrderManager::State> sharedOrderState;
        std::optional<Vehicles::RoutingManager::State> pathReservationState;
        bool discardPathReservationsOnLoad{};
        std::optional<Vehicles::VehicleAutoRenewal::State> vehicleAutoRenewalState;
        std::optional<Vehicles::RailTraffic::State> railTrafficState;
    };
}
