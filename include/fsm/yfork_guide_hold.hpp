#pragma once

#include <chrono>

struct YforkGuideSnapshot
{
    bool valid{false};
    bool expired{false};
    int row{0};
    int column{0};
};

class YforkGuideHold
{
public:
    using Clock = std::chrono::steady_clock;

    YforkGuideSnapshot update(int detectedRow, int detectedColumn,
                              Clock::time_point now)
    {
        if (detectedRow > 0 && detectedColumn > 0)
        {
            row = detectedRow;
            column = detectedColumn;
            lossStartedAt = now;
            lossTimerActive = false;
            return {true, false, row, column};
        }
        if (row <= 0 || column <= 0)
            return {};
        if (!lossTimerActive)
            lossTimerActive = true;
        if (now - lossStartedAt < std::chrono::milliseconds(600))
            return {true, false, row, column};

        reset();
        return {false, true, 0, 0};
    }

    bool recentlyLost(Clock::time_point now,
                      std::chrono::milliseconds duration) const
    {
        return lossTimerActive && now - lossStartedAt < duration;
    }

    void reset() { *this = YforkGuideHold{}; }

private:
    int row{0};
    int column{0};
    bool lossTimerActive{false};
    Clock::time_point lossStartedAt{};
};
