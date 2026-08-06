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

    static void run(FsmPark &park, cv::Mat &img,
                    const std::vector<PredictResult> &results,
                    bool fresh)
    {
        park.params->results = results;
        park.params->aiResultFresh = fresh;
        park.run(img);
    }
};
