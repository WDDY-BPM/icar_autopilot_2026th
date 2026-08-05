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
    MANUAL = 1u << 9, AI_STALE = 1u << 10
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
        append(StopReason::AI_STALE, "AI_STALE");
        return result;
    }
private:
    std::uint32_t bits = 0;
};

} // namespace control_algorithms
