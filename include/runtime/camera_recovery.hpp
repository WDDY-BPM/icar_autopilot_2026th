#pragma once

struct CameraRecoveryUpdate
{
    bool cameraStopActive{false};
    bool controlReady{true};
    int freshFrames{0};
    int holdFrames{0};
};

class CameraRecoveryState
{
public:
    CameraRecoveryUpdate onTimeout()
    {
        cameraStopActive_ = true;
        freshFrames_ = 0;
        holdFrames_ = 0;
        return snapshot();
    }

    CameraRecoveryUpdate onFreshFrame()
    {
        if (cameraStopActive_)
        {
            if (++freshFrames_ >= REQUIRED_FRESH_FRAMES)
            {
                cameraStopActive_ = false;
                freshFrames_ = REQUIRED_FRESH_FRAMES;
                holdFrames_ = RECOVERY_HOLD_FRAMES;
            }
            return snapshot(false);
        }
        if (holdFrames_ > 0)
        {
            --holdFrames_;
            return snapshot(false);
        }
        return snapshot(true);
    }

    CameraRecoveryUpdate snapshot() const
    {
        return snapshot(!cameraStopActive_ && holdFrames_ == 0);
    }

    static constexpr int REQUIRED_FRESH_FRAMES = 3;
    static constexpr int RECOVERY_HOLD_FRAMES = 2;

private:
    CameraRecoveryUpdate snapshot(bool controlReady) const
    {
        return {cameraStopActive_, controlReady, freshFrames_, holdFrames_};
    }

    bool cameraStopActive_{false};
    int freshFrames_{0};
    int holdFrames_{0};
};
