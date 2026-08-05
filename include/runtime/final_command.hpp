#pragma once

struct RequestedCommand
{
    float speed{0.0f};
    int servo{1500};
};

struct SafetySnapshot
{
    bool mustStop{false};
    bool emergency{false};
    bool cameraReady{true};
    bool startupReady{true};
};

struct FinalCommand
{
    float speed{0.0f};
    int servo{1500};
};

inline FinalCommand resolveFinalCommand(const RequestedCommand &requested,
                                        const SafetySnapshot &safety,
                                        int centerServo)
{
    if (safety.mustStop || safety.emergency || !safety.cameraReady ||
        !safety.startupReady)
        return {0.0f, centerServo};
    return {requested.speed, requested.servo};
}
