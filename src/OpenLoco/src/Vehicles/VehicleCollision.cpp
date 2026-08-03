#include "Vehicles/VehicleCollision.h"

namespace OpenLoco::Vehicles::VehicleCollision
{
    static Callback _callback;

    void setCallback(const Callback callback)
    {
        _callback = callback;
    }

    void notify(const EntityId sourceHead, const EntityId collidedComponent)
    {
        if (_callback != nullptr)
        {
            _callback(sourceHead, collidedComponent);
        }
    }
}
