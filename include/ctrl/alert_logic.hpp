#pragma once


namespace control_algorithms
{
inline bool advanceAlertCountdown(int &countdown, int beepEvery = 10)
{
    if (countdown <= 0)
        return false;
    const bool shouldBeep = beepEvery > 0 && countdown % beepEvery == 0;
    countdown--;
    return shouldBeep;
}

inline int updateAlertDecelCountdown(int countdown, bool targetDetected,
                                     int holdFrames = 5)
{
    if (targetDetected)
        return holdFrames;
    return countdown > 0 ? countdown - 1 : 0;
}

struct AlertTimerEvents
{
    bool busyBeep = false;
    bool regularBeep = false;
};

inline void refreshAlertEvidence(bool freshResult, bool targetDetected,
                                 int &alertCountdown, int alertFrames = 30)
{
    if (freshResult && targetDetected && alertCountdown <= 0)
        alertCountdown = alertFrames;
}

inline AlertTimerEvents advanceAlertTimers(int &busyCountdown,
                                           int &alertCountdown,
                                           int &decelCountdown,
                                           bool decelEvidenceRefreshed,
                                           int beepEvery = 10,
                                           int decelHoldFrames = 5)
{
    AlertTimerEvents events;
    events.busyBeep = advanceAlertCountdown(busyCountdown, beepEvery);
    events.regularBeep = advanceAlertCountdown(alertCountdown, beepEvery);
    decelCountdown = updateAlertDecelCountdown(
        decelCountdown, decelEvidenceRefreshed, decelHoldFrames);
    return events;
}

} // namespace control_algorithms
