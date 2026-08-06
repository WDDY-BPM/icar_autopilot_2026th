#pragma once

#include "runtime/path_override.hpp"

struct PlannerSafetyState
{
    bool latched{false};
    PathSource rejectedSource{PathSource::NONE};
    int validRecoveryFrames{0};

    void reject(PathSource source)
    {
        latched = source != PathSource::NONE;
        rejectedSource = source;
        validRecoveryFrames = 0;
    }

    bool observeFrame(bool matchingValidPlan)
    {
        if (!latched)
            return false;
        if (!matchingValidPlan)
        {
            validRecoveryFrames = 0;
            return false;
        }
        if (++validRecoveryFrames < 2)
            return false;
        clear();
        return true;
    }

    bool observe(PathSource source, bool valid)
    {
        return latched && source == rejectedSource && observeFrame(valid);
    }

    void clear(PathSource source = PathSource::NONE)
    {
        if (source != PathSource::NONE && source != rejectedSource)
            return;
        latched = false;
        rejectedSource = PathSource::NONE;
        validRecoveryFrames = 0;
    }
};
