#pragma once

#include <OpenLoco/Types.hpp>

namespace OpenLoco::Vehicles::VehicleCollision
{
    using Callback = void (*)(EntityId sourceHead, EntityId collidedComponent);

    void setCallback(Callback callback);
    void notify(EntityId sourceHead, EntityId collidedComponent);
}
