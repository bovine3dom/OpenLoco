#pragma once

#include <cstdint>

namespace OpenLoco::Scenario
{
    struct Construction
    {
        static constexpr uint8_t kDefaultSignalTrainLength = 5;
        static constexpr uint8_t kMinSignalTrainLength = 1;
        static constexpr uint8_t kMaxSignalTrainLength = 64;

        uint8_t signals[8];       // 0x00015A (0x00525F72)
        uint8_t bridges[8];       // 0x000162 (0x00525F7A)
        uint8_t trainStations[8]; // 0x00016A (0x00525F82)
        uint8_t trackMods[8];     // 0x000172 (0x00525F8A)
        uint8_t var_17A[8];       // 0x00017A (0x00525F92)
        uint8_t roadStations[8];  // 0x000182 (0x00525F9A)
        uint8_t roadMods[8];      // 0x00018A (0x00525FA2)

        uint8_t lastSignalMode() const { return var_17A[0]; }
        void setLastSignalMode(const uint8_t mode) { var_17A[0] = mode; }
        uint8_t lastSignalTrainLength() const
        {
            const auto length = var_17A[1];
            return length >= kMinSignalTrainLength && length <= kMaxSignalTrainLength ? length : kDefaultSignalTrainLength;
        }
        void setLastSignalTrainLength(const uint8_t length)
        {
            var_17A[1] = length;
            if (length < kMinSignalTrainLength)
            {
                var_17A[1] = kMinSignalTrainLength;
            }
            else if (length > kMaxSignalTrainLength)
            {
                var_17A[1] = kMaxSignalTrainLength;
            }
        }
    };

    Construction& getConstruction();
    void resetRoadObjects();
    void resetTrackObjects();

    void initialiseDefaultTrackRoadMods();
}
