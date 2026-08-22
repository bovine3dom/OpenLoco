// SPDX-License-Identifier: MIT
#include "GameRules.h"

#include "SceneManager.h"
#include "World/CompanyManager.h"

namespace OpenLoco::GameRules
{
    namespace
    {
        State _state = kDefaultState;
    }

    void reset()
    {
        _state = kDefaultState;
    }

    State captureState()
    {
        return _state;
    }

    void restoreState(const State& state)
    {
        _state = state;
    }

    bool vehiclesNeverExpire()
    {
        return _state.vehiclesNeverExpire;
    }

    void setVehiclesNeverExpire(const bool value)
    {
        _state.vehiclesNeverExpire = value;
        CompanyManager::determineAvailableVehicles();
    }

    bool extendedVehicleObjects()
    {
        return _state.extendedVehicleObjects;
    }

    bool setExtendedVehicleObjects(const bool value)
    {
        if (!SceneManager::isEditorMode())
        {
            return false;
        }

        _state.extendedVehicleObjects = value;
        return true;
    }
}
