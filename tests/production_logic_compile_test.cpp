#include "ctrl/center.hpp"
#include "ctrl/perception_geometry_builder.hpp"
#include "ctrl/planned_geometry_builder.hpp"
#include "fsm/obstacle.hpp"
#include "fsm/park.hpp"
#include "fsm/yfork.hpp"
#include "park_fsm_test_fixture.hpp"
#include "runtime/path_override.hpp"
#include "test_check.hpp"
#include "test_params.hpp"

#include <memory>
#include <vector>

int main()
{
    // FsmPark::getMode must not reference the removed `step` alias; a link of
    // this executable proves src/fsm/park.cpp compiles with the production
    // source set.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        FsmPark park(params);
        CHECK(park.getMode() == FsmMode::NORMAL);
        ParkFsmTestFixture::setStage(park, ParkFsmTestFixture::Step::ENABLE);
        CHECK(park.getMode() == FsmMode::PARK);
        CHECK(params->ctrl.parking);
        ParkFsmTestFixture::setStage(park, ParkFsmTestFixture::Step::NONE);
        CHECK(park.getMode() == FsmMode::NORMAL);
        CHECK(!params->ctrl.parking);
    }
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = false;
        FsmPark park(params);
        ParkFsmTestFixture::setStage(park, ParkFsmTestFixture::Step::ENABLE);
        CHECK(park.getMode() == FsmMode::NORMAL);
        CHECK(!params->ctrl.parking);
    }
    // FsmObstacle compiles and reports NO_FRESH_RESULT without an AI frame.
    {
        auto params = makeTestParams();
        FsmObstacle obstacle(params);
        params->aiResultFresh = false;
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult == ObstaclePlanningResult::NO_FRESH_RESULT);
    }
    // FsmYfork compiles and stays disabled while the feature is off.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->yfork = false;
        FsmYfork yfork(params);
        CHECK(yfork.getMode() == FsmMode::NORMAL);
    }
    // Center compiles; empty track yields an invalid control geometry.
    {
        auto params = makeTestParams();
        Center center;
        params->geometryPolicy = GeometryPolicy::PERCEPTION_ALLOWED;
        params->mode = FsmMode::NORMAL;
        center.fitting(params);
        CHECK(!center.controlValid);
        CHECK(!center.geometry.valid);
    }
    return 0;
}
