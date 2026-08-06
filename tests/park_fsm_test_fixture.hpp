#pragma once

#include "fsm/park.hpp"

/**
 * White-box fixture for FsmPark. FsmPark declares this struct as a friend so
 * hardware-independent logic tests can drive and inspect the private stage
 * machine without copying production logic.
 */
struct ParkFsmTestFixture
{
    using Step = FsmPark::Step;

    static Step stage(const FsmPark &park) { return park.state.stage; }

    static void setStage(FsmPark &park, Step next)
    {
        park.setStep(next);
    }

    static void reset(FsmPark &park) { park.reset(); }

    static int stageControlFrames(const FsmPark &park)
    {
        return park.state.stageControlFrames;
    }

    static void setSpotsChecked(FsmPark &park, bool checked)
    {
        park.spots.checked = checked;
    }

    static void setSpotEnabled(FsmPark &park, int index, bool enabled)
    {
        if (index >= 0 && index < 4)
            park.spots.spotEnable[index] = enabled;
    }

    static void setForks(FsmPark &park, std::vector<PredictResult> forks)
    {
        park.spots.forks = std::move(forks);
    }

    static void setCountOut(FsmPark &park, int value) { park.countOut = value; }

    static int countOut(const FsmPark &park) { return park.countOut; }

    static void setStageStartedAt(
        FsmPark &park, std::chrono::steady_clock::time_point timePoint)
    {
        park.state.stageStartedAt = timePoint;
    }

    static void setForkOutStartedAt(
        FsmPark &park, std::chrono::steady_clock::time_point timePoint)
    {
        park.state.forkOutStartedAt = timePoint;
    }

    static int exitSignMissing(const FsmPark &park)
    {
        return park.state.exitSignMissingConfirmations;
    }

    static std::size_t replayHistorySize(const FsmPark &park)
    {
        return park.pointsEdgeLeftPast.size();
    }

    static void clearReplayHistory(FsmPark &park)
    {
        park.pointsEdgeLeftPast.clear();
        park.pointsEdgeRightPast.clear();
    }

    static void run(FsmPark &park, cv::Mat &img,
                    const std::vector<PredictResult> &results,
                    bool fresh)
    {
        park.params->results = results;
        park.params->aiResultFresh = fresh;
        park.run(img);
    }
};
