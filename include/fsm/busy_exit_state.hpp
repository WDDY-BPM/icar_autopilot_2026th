#pragma once

#include <chrono>

enum class BusyExitEvent
{
    NONE,
    EXIT_STARTED,
    SIGN_WAIT_TIMEOUT,
    EXIT_GUIDE_TIMEOUT,
    COMPLETED
};

class BusyExitState
{
public:
    using Clock = std::chrono::steady_clock;

    void startDriving(Clock::time_point now)
    {
        reset();
        driving = true;
        drivingStartedAt = now;
    }

    BusyExitEvent update(bool signSearchEnabled, bool freshAi,
                         bool leftSignVisible, Clock::time_point now)
    {
        if (!driving || stopped)
            return BusyExitEvent::NONE;
        if (!signSearchEnabled)
        {
            signWaitStarted = false;
            return BusyExitEvent::NONE;
        }
        if (!signWaitStarted)
        {
            signWaitStarted = true;
            exitSignWaitStartedAt = now;
        }
        if (!exiting)
        {
            if (freshAi && leftSignVisible)
            {
                exiting = true;
                exitStartedAt = now;
                consecutiveMissing = 0;
                return BusyExitEvent::EXIT_STARTED;
            }
            if (now - exitSignWaitStartedAt >= std::chrono::seconds(8))
            {
                stopped = true;
                return BusyExitEvent::SIGN_WAIT_TIMEOUT;
            }
            return BusyExitEvent::NONE;
        }

        if (freshAi)
            consecutiveMissing = leftSignVisible ? 0 : consecutiveMissing + 1;
        if (consecutiveMissing >= 3)
        {
            driving = false;
            exiting = false;
            return BusyExitEvent::COMPLETED;
        }
        if (now - exitStartedAt >= std::chrono::seconds(2))
        {
            stopped = true;
            exiting = false;
            return BusyExitEvent::EXIT_GUIDE_TIMEOUT;
        }
        return BusyExitEvent::NONE;
    }

    void reset() { *this = BusyExitState{}; }

    bool driving{false};
    bool exiting{false};
    bool stopped{false};
    bool signWaitStarted{false};
    int consecutiveMissing{0};
    Clock::time_point drivingStartedAt{};
    Clock::time_point exitSignWaitStartedAt{};
    Clock::time_point exitStartedAt{};
};
