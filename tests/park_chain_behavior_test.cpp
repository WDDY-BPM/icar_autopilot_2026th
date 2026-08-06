#include "ctrl/center.hpp"
#include "fsm/yfork.hpp"
#include "park_fsm_test_fixture.hpp"
#include "runtime/control_decision.hpp"
#include "test_check.hpp"
#include "test_params.hpp"

#include <chrono>
#include <thread>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;
using ParkStep = ParkFsmTestFixture::Step;

PredictResult parkMarker(int x = 200, int y = 80)
{
    return PredictResult{LABEL_PARK, "", 0.9f, x, y, 40, 40};
}

PredictResult choiceMarker()
{
    return PredictResult{LABEL_CHOICE, "", 0.9f, 120, 90, 30, 30};
}

PredictResult forkMarker(int y, int height = 20)
{
    return PredictResult{LABEL_FORK, "", 0.9f, 120, y, 30, height};
}
} // namespace

int main()
{
    // ==== PARK full chain driven through the test fixture ====
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        params->config.currentLapConfig->parkSpot = 4;
        FsmPark park(params);
        cv::Mat img;

        // NONE -> ENABLE: 3 fresh frames with a valid parking marker.
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::NONE);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::ENABLE);

        // ENABLE -> FORKIN: 2 fresh frames with an approaching choice marker.
        ParkFsmTestFixture::run(park, img, {choiceMarker()}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::ENABLE);
        ParkFsmTestFixture::run(park, img, {choiceMarker()}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::FORKIN);

        // FORKIN: PLANNED_REQUIRED with a legal PARK path; then timeout to
        // TRACKIN.
        ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(params->geometryPolicy == GeometryPolicy::PLANNED_REQUIRED);
        CHECK(params->pathOverride.validFor(PathSource::PARK));
        for (int frame = 0; frame < 24; ++frame)
            ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::TRACKIN);

        // TRACKIN: fresh AI guidance builds a TIME_TTL PARK path; stale AI
        // frames must not clear it immediately and PERCEPTION stays allowed.
        CHECK(params->geometryPolicy == GeometryPolicy::PERCEPTION_ALLOWED);
        ParkFsmTestFixture::run(park, img, {forkMarker(100)}, true);
        CHECK(params->pathOverride.validFor(PathSource::PARK));
        CHECK(params->pathOverride.freshnessMode ==
              PathFreshnessMode::TIME_TTL);
        ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(params->pathOverride.validFor(PathSource::PARK));
        std::this_thread::sleep_for(std::chrono::milliseconds(180));
        ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(!params->pathOverride.validFor(PathSource::PARK));
        CHECK(params->geometryPolicy == GeometryPolicy::PERCEPTION_ALLOWED);

        // TRACKIN -> ENTER: checked spots with a far fork crossing the
        // spotUp threshold.
        ParkFsmTestFixture::setSpotsChecked(park, true);
        ParkFsmTestFixture::setSpotEnabled(park, 0, true);
        ParkFsmTestFixture::setForks(park, {forkMarker(110, 20), forkMarker(160, 20)});
        ParkFsmTestFixture::run(park, img, {forkMarker(110, 20), forkMarker(160, 20)}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::ENTER);

        // ENTER: PLANNED_REQUIRED, records dual-edge paths, timeout -> PARKING.
        CHECK(params->geometryPolicy == GeometryPolicy::PLANNED_REQUIRED);
        for (int frame = 0; frame < 31; ++frame)
            ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::PARKING);
        CHECK(ParkFsmTestFixture::replayHistorySize(park) > 0);
        CHECK(params->hasStopReason(control_algorithms::StopReason::PARK));
        CHECK(params->geometryPolicy == GeometryPolicy::STOPPED);

        // PARKING: keep PARK stop; advance time to WAIT_PICKUP.
        ParkFsmTestFixture::setStageStartedAt(
            park, Clock::now() - std::chrono::milliseconds(800));
        ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::WAIT_PICKUP);
        CHECK(params->hasStopReason(control_algorithms::StopReason::PARK));
        CHECK(params->geometryPolicy == GeometryPolicy::STOPPED);

        // WAIT_PICKUP: keep PARK stop; advance time to EXIT.
        ParkFsmTestFixture::setStageStartedAt(
            park, Clock::now() - std::chrono::seconds(4));
        ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::EXIT);
        CHECK(!params->hasStopReason(control_algorithms::StopReason::PARK));

        // EXIT handler: reverse flag on, replay path built from history.
        ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(params->ctrl.back);
        CHECK(params->pathOverride.validFor(PathSource::PARK));

        // EXIT: when the replay history is exhausted -> TRACKOUT.
        ParkFsmTestFixture::clearReplayHistory(park);
        ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::TRACKOUT);

        // TRACKOUT: PERCEPTION allowed, guidance offset 0, timeout -> FORKOUT.
        CHECK(params->geometryPolicy == GeometryPolicy::PERCEPTION_ALLOWED);
        ParkFsmTestFixture::run(park, img, {forkMarker(100)}, true);
        CHECK(params->pathOverride.validFor(PathSource::PARK));
        const int expectedEndColumn =
            forkMarker(100).x + forkMarker(100).width / 2;
        CHECK(params->pathOverride.centerLine.back().y == expectedEndColumn);
        for (int frame = 0; frame < 49; ++frame)
            ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::FORKOUT);

        // FORKOUT: only continuous left-sign disappearance completes the lap.
        CHECK(params->geometryPolicy == GeometryPolicy::PLANNED_REQUIRED);
        params->lapTaskRequired = true;
        params->lapTaskCompleted = false;
        for (int frame = 0; frame < 3; ++frame)
            ParkFsmTestFixture::run(park, img, {}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::NONE);
        CHECK(params->lapTaskCompleted);
        CHECK(params->ctrl.yforkReset);
        CHECK(params->ctrl.countAcc == params->config.startupRampFrames);
    }

    // FORKOUT 2s timeout must NOT complete the lap task.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        params->lapTaskRequired = true;
        params->lapTaskCompleted = false;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::setStage(park, ParkStep::FORKOUT);
        ParkFsmTestFixture::setForkOutStartedAt(
            park, Clock::now() - std::chrono::milliseconds(2100));
        ParkFsmTestFixture::run(park, img, {}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::NONE);
        CHECK(!params->lapTaskCompleted);
    }

    // ENABLE timeout resets immediately and must not advance to FORKIN in
    // the same frame.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::setStage(park, ParkStep::ENABLE);
        for (int frame = 0; frame < 81; ++frame)
            ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::NONE);
    }

    // NONE: stale frames must not accumulate parking-marker evidence and a
    // 5-frame fresh-missing streak clears the confirmation counter.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::run(park, img, {parkMarker()}, false);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, false);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::NONE);
        for (int frame = 0; frame < 5; ++frame)
            ParkFsmTestFixture::run(park, img, {}, true);
        // The clear worked: one marker is not enough to enable again.
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::NONE);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        ParkFsmTestFixture::run(park, img, {parkMarker()}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::ENABLE);
    }

    // ==== Full chain: PARK TRACKIN AI gap lets PERCEPTION drive ====
    {
        auto params = makeTestParams();
        params->mode = FsmMode::PARK;
        params->geometryPolicy = GeometryPolicy::PERCEPTION_ALLOWED;
        setStraightTrack(params, 220, 40, 80, 240);
        params->track->quality.valid = true;
        params->track->quality.leftReliable = true;
        params->track->quality.rightReliable = true;
        params->track->quality.widthVariation = 0.05f;
        Center center;
        center.fitting(params);
        CHECK(center.controlValid);
        CHECK(center.geometry.source == ControlGeometrySource::PERCEPTION);
        CHECK(evaluateRuntimeControl(
            center.geometry, params->stopReasons, true).allowMotion);
    }

    // ==== Full chain: PARK ENTER without a planned path must stop ====
    {
        auto params = makeTestParams();
        params->mode = FsmMode::PARK;
        params->geometryPolicy = GeometryPolicy::PLANNED_REQUIRED;
        setStraightTrack(params, 220, 40, 80, 240);
        Center center;
        center.fitting(params);
        CHECK(!center.controlValid);
        CHECK(!center.geometry.valid);
        const auto decision = evaluateRuntimeControl(
            center.geometry, params->stopReasons, true);
        CHECK(!decision.allowMotion);
        CHECK(decision.centerSteering);
    }

    // ==== Full chain: YFORK recovery stage falls back to PERCEPTION ====
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = false;
        params->config.currentLapConfig->yfork = true;
        params->config.currentLapConfig->yforkLeft = false;
        params->config.currentLapConfig->station = false;
        FsmYfork yfork(params);
        cv::Mat img = cv::Mat::zeros(cv::Size(COLSIMAGE, ROWSIMAGE), CV_8U);

        params->results = {PredictResult{LABEL_FORK, "", 0.9f, 100, 100, 40, 40}};
        params->aiResultFresh = true;
        yfork.run(img);
        CHECK(params->yforkPhase == YforkRuntimePhase::GUIDE_ACTIVE);
        CHECK(yfork.getMode() == FsmMode::YFORK);

        params->results.clear();
        params->track->spurroad = {PointX(150, 100)};
        yfork.run(img); // find V-tip -> DECIDE
        yfork.run(img); // DECIDE -> ENTER (PLANNED_REQUIRED)
        CHECK(params->geometryPolicy == GeometryPolicy::PLANNED_REQUIRED);

        params->track->spurroad.clear();
        yfork.run(img); // starts the guide-loss timer
        std::this_thread::sleep_for(std::chrono::milliseconds(650));
        yfork.run(img); // guide expired -> PERCEPTION_RECOVERY
        CHECK(params->yforkPhase == YforkRuntimePhase::PERCEPTION_RECOVERY);
        CHECK(params->geometryPolicy == GeometryPolicy::PERCEPTION_ALLOWED);
        CHECK(!params->pathOverride.active());
    }
    return 0;
}
