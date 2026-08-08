#include "ctrl/lane_control.hpp"
#include "ctrl/motion.hpp"
#include "ctrl/perception_geometry_builder.hpp"
#include "test_check.hpp"
#include "test_params.hpp"

#include <cmath>
#include <vector>

namespace
{
bool closeTo(float value, float expected, float tolerance = 1e-3f)
{
    return std::abs(value - expected) <= tolerance;
}

std::vector<PointX> straddleCenterline()
{
    // near=170 / far=150 -> oppositeSideRecovery（跨图像中心）。
    std::vector<PointX> centerline;
    for (int row = 120; row <= 175; ++row)
        centerline.emplace_back(row, 150);
    for (int row = 180; row <= 220; ++row)
        centerline.emplace_back(row, 170);
    return centerline;
}
} // namespace

int main()
{
    // 1. lateralScaleForMode：单边缩放，其余模式不缩放。
    {
        auto params = makeTestParams();
        CHECK(lateralScaleForMode(LaneRecoveryMode::STRICT_DUAL,
                                  params->config) == 1.0f);
        CHECK(lateralScaleForMode(LaneRecoveryMode::RELAXED_DUAL,
                                  params->config) == 1.0f);
        CHECK(lateralScaleForMode(LaneRecoveryMode::WEAK_HYBRID,
                                  params->config) == 1.0f);
        CHECK(lateralScaleForMode(LaneRecoveryMode::INVALID,
                                  params->config) == 1.0f);
        CHECK(lateralScaleForMode(LaneRecoveryMode::LEFT_SINGLE,
                                  params->config) ==
              params->config.singleLaneHeadingConfidence);
        CHECK(lateralScaleForMode(LaneRecoveryMode::RIGHT_SINGLE,
                                  params->config) ==
              params->config.singleLaneHeadingConfidence);
    }
    // 2. recovery damping 开关：仅真实双边模式启用 0.25。
    {
        CHECK(recoveryDampingEnabledForMode(LaneRecoveryMode::STRICT_DUAL));
        CHECK(recoveryDampingEnabledForMode(LaneRecoveryMode::RELAXED_DUAL));
        CHECK(!recoveryDampingEnabledForMode(LaneRecoveryMode::WEAK_HYBRID));
        CHECK(!recoveryDampingEnabledForMode(LaneRecoveryMode::LEFT_SINGLE));
        CHECK(!recoveryDampingEnabledForMode(LaneRecoveryMode::RIGHT_SINGLE));
    }
    // 3. STRICT_DUAL：lateral 不缩放（scale=1.0），heading 原样叠加。
    {
        auto params = makeTestParams();
        params->ctrl.center = 170; // error=+10
        params->ctrl.laneHeadingCorrection = -20.0f;
        params->ctrl.laneLateralScale = 1.0f;
        Motion motion;
        motion.reset();
        motion.poseControl(params, 0.03f);
        CHECK(closeTo(params->ctrl.lateralRaw, 22.6f));
        CHECK(closeTo(params->ctrl.lateralApplied, 22.6f));
        CHECK(closeTo(params->ctrl.headingApplied, -20.0f));
        CHECK(params->ctrl.pwmDiff == 3);
        CHECK(params->ctrl.servo == 1497);
    }
    // 4. RIGHT_SINGLE：lateral 缩放，heading 不缩放。
    {
        auto params = makeTestParams();
        params->ctrl.center = 170;
        params->ctrl.laneHeadingCorrection = -20.0f;
        params->ctrl.laneLateralScale =
            params->config.singleLaneHeadingConfidence; // 0.45
        Motion motion;
        motion.reset();
        motion.poseControl(params, 0.03f);
        CHECK(closeTo(params->ctrl.lateralRaw, 22.6f));
        CHECK(closeTo(params->ctrl.lateralApplied, 22.6f * 0.45f));
        CHECK(closeTo(params->ctrl.headingApplied, -20.0f)); // 不缩放
        CHECK(params->ctrl.pwmDiff == -10);
        CHECK(params->ctrl.servo == 1510);
    }
    // 5. LEFT_SINGLE 与 RIGHT_SINGLE 严格镜像。
    {
        auto params = makeTestParams();
        params->ctrl.center = 150; // error=-10
        params->ctrl.laneHeadingCorrection = 20.0f;
        params->ctrl.laneLateralScale =
            params->config.singleLaneHeadingConfidence;
        Motion motion;
        motion.reset();
        motion.poseControl(params, 0.03f);
        CHECK(closeTo(params->ctrl.lateralApplied, -22.6f * 0.45f));
        CHECK(closeTo(params->ctrl.headingApplied, 20.0f));
        CHECK(params->ctrl.pwmDiff == 10);
        CHECK(params->ctrl.servo == 1490);
    }
    // 6. WEAK_HYBRID：跨中心几何不再被 0.25 额外削弱。
    {
        const auto centerline = straddleCenterline();
        const auto damped = control_algorithms::calculateLaneControlCenters(
            centerline, 160.0f, 0.65f, 8, true, 300.0f, 60.0f, 40.0f,
            1.0f, true);
        const auto undamped = control_algorithms::calculateLaneControlCenters(
            centerline, 160.0f, 0.65f, 8, true, 300.0f, 60.0f, 40.0f,
            1.0f, false);
        CHECK(damped.oppositeSideRecovery);
        CHECK(closeTo(damped.recoveryDamping, 0.25f));
        CHECK(undamped.oppositeSideRecovery);
        CHECK(closeTo(undamped.recoveryDamping, 1.0f));
    }
    // 7. 单边 center_error 很大时，pwmDiff 不被 lateral 项无上限主导。
    {
        auto params = makeTestParams();
        params->ctrl.center = 80; // error=-80
        params->ctrl.laneHeadingCorrection = -35.0f;
        params->ctrl.laneLateralScale =
            params->config.singleLaneHeadingConfidence;
        Motion motion;
        motion.reset();
        motion.poseControl(params, 0.03f);
        CHECK(closeTo(params->ctrl.lateralRaw, -214.4f, 1e-2f));
        CHECK(closeTo(params->ctrl.lateralApplied, -214.4f * 0.45f, 1e-2f));
        CHECK(closeTo(params->ctrl.headingApplied, -35.0f));
        CHECK(std::abs(params->ctrl.pwmDiff) <
              std::abs(params->ctrl.lateralRaw));
        CHECK(params->ctrl.servo >= 1100 && params->ctrl.servo <= 1900);
    }
    // 8. headingCorrection 减小时，同样 center_error 下舵机应回舵。
    {
        auto params = makeTestParams();
        params->ctrl.center = 170;
        params->ctrl.laneLateralScale =
            params->config.singleLaneHeadingConfidence;
        Motion motion;
        motion.reset();
        params->ctrl.laneHeadingCorrection = -60.0f;
        // 连续几帧让舵机越过速率限幅并稳定到目标值。
        for (int frame = 0; frame < 3; ++frame)
            motion.poseControl(params, 0.03f);
        const int servoStrongHeading = params->ctrl.servo;
        params->ctrl.laneHeadingCorrection = -30.0f;
        motion.poseControl(params, 0.03f);
        const int servoWeakerHeading = params->ctrl.servo;
        CHECK(servoStrongHeading > 1500);
        CHECK(servoWeakerHeading > 1500);
        CHECK(servoWeakerHeading < servoStrongHeading);
    }
    // 9. laneHeadingMaxCorrection 限幅保持。
    {
        std::vector<PointX> steep;
        for (int row = 120; row <= 175; ++row)
            steep.emplace_back(row, 100);
        for (int row = 180; row <= 220; ++row)
            steep.emplace_back(row, 200);
        const auto result = control_algorithms::calculateLaneControlCenters(
            steep, 160.0f, 0.65f, 8, true, 300.0f, 60.0f, 40.0f, 1.0f,
            false);
        CHECK(std::abs(result.headingCorrection) <= 60.0f);
    }
    // 10. servo 最大/最小限幅保持。
    {
        Motion motion;
        motion.syncServoCommand(1900);
        CHECK(motion.limitServoCommand(5000, 0.03f) == 1900);
        motion.syncServoCommand(1100);
        CHECK(motion.limitServoCommand(-5000, 0.03f) == 1100);
    }
    return 0;
}
