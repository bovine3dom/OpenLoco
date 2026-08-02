#pragma once

#include "GameCommands/GameCommands.h"
#include "Map/TrackElement.h"

namespace OpenLoco::GameCommands
{
    struct SignalPlacementArgs
    {
        static constexpr auto command = GameCommand::createSignal;

        SignalPlacementArgs() = default;
        explicit SignalPlacementArgs(const registers& regs)
            : pos(regs.ax, regs.cx, regs.di)
            , rotation(regs.bh & 0x3)
            , trackId(regs.dl & 0x3F)
            , index(regs.dh)
            , type((regs.edi >> 16) & 0xFF)
            , mode(World::sanitiseSignalMode((regs.edi >> 24) & 0x3))
            , trackObjType(regs.ebp & 0xFF)
            , sides((regs.edi >> 16) & 0xC000)
        {
        }

        World::Pos3 pos;
        uint8_t rotation;
        uint8_t trackId;
        uint8_t index;
        uint8_t type;
        World::SignalMode mode = World::SignalMode::block;
        uint8_t trackObjType;
        uint16_t sides;

        explicit operator registers() const
        {
            registers regs;
            regs.ax = pos.x;
            regs.cx = pos.y;
            regs.bh = rotation;
            regs.dl = trackId;
            regs.dh = index;
            regs.edi = pos.z | (type << 16) | (static_cast<uint8_t>(mode) << 24) | (static_cast<uint32_t>(sides & 0xC000) << 16);
            regs.ebp = trackObjType;
            return regs;
        }
    };

    void createSignal(registers& regs, const uint8_t flags);
}
