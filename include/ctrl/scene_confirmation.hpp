#pragma once

#include <cstdint>

namespace control_algorithms
{
constexpr int BUSY_CONFIRM_POSITIVE_FRAMES = 4;
constexpr int BUSY_CLEAR_NEGATIVE_FRAMES = 5;
constexpr std::int64_t AI_STALE_TIMEOUT_MS = 500;
constexpr int AI_RECOVERY_FRESH_RESULTS = 3;
constexpr int CROSS_DISAPPEAR_FRAMES = 15;

struct BusyConfirmationState
{
    int positiveFrames = 0;
    int negativeFrames = 0;
    bool waiting = false;
    bool confirmed = false;
};

enum class BusyConfirmationEvent
{
    NONE,
    WAITING,
    CONFIRMED,
    CLEARED
};

inline BusyConfirmationEvent updateBusyConfirmation(
    BusyConfirmationState &state, bool freshResult, bool busyDetected)
{
    if (!freshResult || state.confirmed)
        return BusyConfirmationEvent::NONE;
    if (busyDetected)
    {
        state.negativeFrames = 0;
        state.positiveFrames++;
        if (state.positiveFrames >= BUSY_CONFIRM_POSITIVE_FRAMES)
        {
            state.confirmed = true;
            state.waiting = false;
            return BusyConfirmationEvent::CONFIRMED;
        }
        if (state.positiveFrames == BUSY_CONFIRM_POSITIVE_FRAMES - 1)
        {
            state.waiting = true;
            return BusyConfirmationEvent::WAITING;
        }
        return BusyConfirmationEvent::NONE;
    }
    if (state.waiting)
    {
        state.negativeFrames++;
        if (state.negativeFrames >= BUSY_CLEAR_NEGATIVE_FRAMES)
        {
            state = BusyConfirmationState{};
            return BusyConfirmationEvent::CLEARED;
        }
    }
    else
    {
        state.positiveFrames = 0;
    }
    return BusyConfirmationEvent::NONE;
}

struct AiFreshnessState
{
    bool automaticActive = false;
    bool automaticSeen = false;
    bool stale = false;
    bool hasSuccessfulResult = false;
    int recoveryFreshResults = 0;
    std::int64_t automaticStartedMs = 0;
    std::int64_t lastSuccessfulResultMs = 0;
};

inline bool updateAiFreshness(AiFreshnessState &state, bool automaticMode,
                              bool successfulFreshResult, std::int64_t nowMs,
                              std::int64_t timeoutMs = AI_STALE_TIMEOUT_MS,
                              int recoveryResults = AI_RECOVERY_FRESH_RESULTS,
                              std::int64_t successfulResultMs = -1)
{
    if (!automaticMode)
    {
        state.automaticActive = false;
        state.stale = false;
        state.recoveryFreshResults = 0;
        return false;
    }
    if (!state.automaticActive)
    {
        const bool returningFromManual = state.automaticSeen;
        state.automaticActive = true;
        state.automaticSeen = true;
        state.automaticStartedMs = nowMs;
        state.recoveryFreshResults = 0;
        state.stale = returningFromManual;
    }
    if (successfulFreshResult)
    {
        const std::int64_t publishedMs = successfulResultMs >= 0
            ? successfulResultMs : nowMs;
        state.hasSuccessfulResult = true;
        state.lastSuccessfulResultMs = publishedMs;
        if (nowMs - publishedMs > timeoutMs)
        {
            state.stale = true;
            state.recoveryFreshResults = 0;
        }
        else if (state.stale && ++state.recoveryFreshResults >= recoveryResults)
        {
            state.stale = false;
            state.recoveryFreshResults = 0;
        }
    }
    else
    {
        const std::int64_t reference = state.hasSuccessfulResult
            ? state.lastSuccessfulResultMs : state.automaticStartedMs;
        if (nowMs - reference > timeoutMs)
        {
            state.stale = true;
            state.recoveryFreshResults = 0;
        }
    }
    return state.stale;
}

struct CrossConfirmationState
{
    bool armed = false;
    bool linePassed = false;
    int passFrames = 0;
    int missingFrames = 0;
};

enum class CrossConfirmationEvent
{
    NONE,
    LAP_PASSED,
    FINAL_STOP
};

inline CrossConfirmationEvent updateCrossConfirmation(
    CrossConfirmationState &state, bool freshResult, bool crossDetected,
    bool passCandidate, bool finalLap, bool lapTaskComplete,
    int passConfirmFrames = 2,
    int disappearFrames = CROSS_DISAPPEAR_FRAMES)
{
    if (!freshResult)
        return CrossConfirmationEvent::NONE;
    if (!state.armed)
    {
        state.missingFrames = crossDetected ? 0 : state.missingFrames + 1;
        if (state.missingFrames >= disappearFrames)
        {
            state.armed = true;
            state.missingFrames = 0;
        }
        return CrossConfirmationEvent::NONE;
    }
    if (!state.linePassed)
    {
        state.passFrames = passCandidate ? state.passFrames + 1 : 0;
        if (state.passFrames >= passConfirmFrames)
        {
            // Only credit a crossing when the lap task was already complete
            // while the line was actually being passed. Disarm until this
            // same line disappears to prevent retroactive lap completion.
            if (!lapTaskComplete)
            {
                state = CrossConfirmationState{};
                return CrossConfirmationEvent::NONE;
            }
            state.linePassed = true;
            state.missingFrames = 0;
        }
        return CrossConfirmationEvent::NONE;
    }
    state.missingFrames = crossDetected ? 0 : state.missingFrames + 1;
    if (state.missingFrames < disappearFrames)
        return CrossConfirmationEvent::NONE;
    const auto event = finalLap
        ? CrossConfirmationEvent::FINAL_STOP
        : CrossConfirmationEvent::LAP_PASSED;
    state = CrossConfirmationState{};
    return event;
}
} // namespace control_algorithms
