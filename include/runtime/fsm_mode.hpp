#pragma once

enum class FsmMode
{
    NORMAL,
    FORK,
    PARK,
    BUSY,
    BUSY_WAIT,
    MANUAL,
    SLOW,
    STOP,
    CROSS,
    YFORK,
    STATION,
};

inline const char *fsmModeName(FsmMode mode)
{
    switch (mode)
    {
    case FsmMode::NORMAL: return "NORMAL";
    case FsmMode::FORK: return "FORK";
    case FsmMode::PARK: return "PARK";
    case FsmMode::BUSY: return "BUSY";
    case FsmMode::BUSY_WAIT: return "BUSY_WAIT";
    case FsmMode::MANUAL: return "MANUAL";
    case FsmMode::SLOW: return "SLOW";
    case FsmMode::STOP: return "STOP";
    case FsmMode::CROSS: return "CROSS";
    case FsmMode::YFORK: return "YFORK";
    case FsmMode::STATION: return "STATION";
    default: return "UNKNOWN";
    }
}
