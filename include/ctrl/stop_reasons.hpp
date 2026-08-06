#pragma once

#include <cstdint>
#include <string>

namespace control_algorithms
{
enum class StopReason : std::uint32_t
{
    STARTUP = 1u << 0, CAMERA = 1u << 1, EMERGENCY = 1u << 2,
    LANE = 1u << 3, GATE = 1u << 4, PARK = 1u << 5,
    BUSY = 1u << 6, STATION = 1u << 7, CROSS = 1u << 8,
    MANUAL = 1u << 9, AI_STALE = 1u << 10, PLANNER = 1u << 11,
    PARK_GATE = 1u << 12, PARK_TARGET_LOST = 1u << 13,
    PARK_ENTER_UNCONFIRMED = 1u << 14,
    PARK_EXIT_UNCONFIRMED = 1u << 15
};

class StopReasonState
{
public:
    void set(StopReason reason, bool active)
    {
        const auto bit = static_cast<std::uint32_t>(reason);
        bits = active ? bits | bit : bits & ~bit;
    }
    bool has(StopReason reason) const
    {
        return (bits & static_cast<std::uint32_t>(reason)) != 0;
    }
    bool mustStop() const { return bits != 0; }
    bool mustStopExcept(StopReason reason) const
    {
        return (bits & ~static_cast<std::uint32_t>(reason)) != 0;
    }
    bool hasOnly(StopReason first, StopReason second) const
    {
        const auto allowed = static_cast<std::uint32_t>(first) |
                             static_cast<std::uint32_t>(second);
        return bits != 0 && (bits & ~allowed) == 0;
    }
    std::uint32_t value() const { return bits; }
    std::string string() const
    {
        if (!bits) return "NONE";
        std::string result;
        const auto append = [&](StopReason reason, const char *name) {
            if (!has(reason)) return;
            if (!result.empty()) result += "|";
            result += name;
        };
        append(StopReason::STARTUP, "STARTUP"); append(StopReason::CAMERA, "CAMERA");
        append(StopReason::EMERGENCY, "EMERGENCY"); append(StopReason::LANE, "LANE");
        append(StopReason::GATE, "GATE"); append(StopReason::PARK, "PARK");
        append(StopReason::BUSY, "BUSY"); append(StopReason::STATION, "STATION");
        append(StopReason::CROSS, "CROSS"); append(StopReason::MANUAL, "MANUAL");
        append(StopReason::AI_STALE, "AI_STALE"); append(StopReason::PLANNER, "PLANNER");
        append(StopReason::PARK_GATE, "PARK_GATE");
        append(StopReason::PARK_TARGET_LOST, "PARK_TARGET_LOST");
        append(StopReason::PARK_ENTER_UNCONFIRMED, "PARK_ENTER_UNCONFIRMED");
        append(StopReason::PARK_EXIT_UNCONFIRMED, "PARK_EXIT_UNCONFIRMED");
        return result;
    }
private:
    std::uint32_t bits = 0;
};

} // namespace control_algorithms
