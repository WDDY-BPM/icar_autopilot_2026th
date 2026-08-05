#pragma once

#include <chrono>

enum class BusyTraversalPhase
{
    IDLE,
    WAIT_STATION,
    WAIT_EXIT_SIGN,
    EXIT_GUIDANCE,
    STOPPED,
    COMPLETED
};

enum class BusyExitEvent
{
    NONE,
    EXIT_STARTED,
    STATION_WAIT_TIMEOUT,
    SIGN_WAIT_TIMEOUT,
    EXIT_GUIDE_TIMEOUT,
    TRAVERSAL_TIMEOUT,
    COMPLETED
};

class BusyExitState
{
public:
    using Clock = std::chrono::steady_clock;

    void startDriving(Clock::time_point now, bool waitForStation = false)
    {
        reset();
        driving = true;
        drivingStartedAt = now;
        phaseStartedAt = now;
        phase = waitForStation ? BusyTraversalPhase::WAIT_STATION
                               : BusyTraversalPhase::WAIT_EXIT_SIGN;
    }

    BusyExitEvent update(bool stationRequired, bool stationCompleted,
                         bool freshAi, bool leftSignVisible,
                         Clock::time_point now)
    {
        if (!driving || stopped)
            return BusyExitEvent::NONE;
        if (now - drivingStartedAt >= std::chrono::seconds(15))
            return stop(BusyExitEvent::TRAVERSAL_TIMEOUT);

        if (phase == BusyTraversalPhase::WAIT_STATION)
        {
            if (!stationRequired || stationCompleted)
            {
                phase = BusyTraversalPhase::WAIT_EXIT_SIGN;
                phaseStartedAt = now;
                return BusyExitEvent::NONE;
            }
            if (now - phaseStartedAt >= std::chrono::seconds(10))
                return stop(BusyExitEvent::STATION_WAIT_TIMEOUT);
            return BusyExitEvent::NONE;
        }

        if (phase == BusyTraversalPhase::WAIT_EXIT_SIGN)
        {
            if (freshAi && leftSignVisible)
            {
                phase = BusyTraversalPhase::EXIT_GUIDANCE;
                phaseStartedAt = now;
                exiting = true;
                consecutiveMissing = 0;
                return BusyExitEvent::EXIT_STARTED;
            }
            if (now - phaseStartedAt >= std::chrono::seconds(8))
                return stop(BusyExitEvent::SIGN_WAIT_TIMEOUT);
            return BusyExitEvent::NONE;
        }

        if (phase == BusyTraversalPhase::EXIT_GUIDANCE)
        {
            if (freshAi)
                consecutiveMissing = leftSignVisible ? 0 : consecutiveMissing + 1;
            if (consecutiveMissing >= 3)
            {
                driving = false;
                exiting = false;
                phase = BusyTraversalPhase::COMPLETED;
                return BusyExitEvent::COMPLETED;
            }
            if (now - phaseStartedAt >= std::chrono::seconds(2))
                return stop(BusyExitEvent::EXIT_GUIDE_TIMEOUT);
        }
        return BusyExitEvent::NONE;
    }

    void reset() { *this = BusyExitState{}; }

    bool driving{false};
    bool exiting{false};
    bool stopped{false};
    int consecutiveMissing{0};
    BusyTraversalPhase phase{BusyTraversalPhase::IDLE};
    Clock::time_point drivingStartedAt{};
    Clock::time_point phaseStartedAt{};

private:
    BusyExitEvent stop(BusyExitEvent event)
    {
        stopped = true;
        exiting = false;
        phase = BusyTraversalPhase::STOPPED;
        return event;
    }
};
