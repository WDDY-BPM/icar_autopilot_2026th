#include "ctrl/center.hpp"
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
    // 1. 动态 singleLaneLateralScale：非单边 full；单边按阶段变化。
    {
        auto params = makeTestParams();
        const auto dual = singleLaneLateralScale(
            LaneRecoveryMode::STRICT_DUAL, true, true, 5.0f, -40.0f, 1.0f,
            params->config);
        CHECK(closeTo(dual.scale, 1.0f));
        CHECK(dual.reason == LateralScaleReason::FULL_DUAL);
        const auto relaxed = singleLaneLateralScale(
            LaneRecoveryMode::RELAXED_DUAL, true, true, 5.0f, -40.0f, 1.0f,
            params->config);
        CHECK(closeTo(relaxed.scale, 1.0f));
        const auto hybrid = singleLaneLateralScale(
            LaneRecoveryMode::WEAK_HYBRID, true, true, 5.0f, -40.0f, 1.0f,
            params->config);
        CHECK(closeTo(hybrid.scale, 1.0f));
        const auto invalid = singleLaneLateralScale(
            LaneRecoveryMode::INVALID, true, true, 5.0f, -40.0f, 1.0f,
            params->config);
        CHECK(closeTo(invalid.scale, 1.0f));

        // RIGHT_SINGLE 左弯预瞄：near=+5 / far=-40 -> 0.45。
        const auto rightPreview = singleLaneLateralScale(
            LaneRecoveryMode::RIGHT_SINGLE, true, true, 5.0f, -40.0f, 1.0f,
            params->config);
        CHECK(closeTo(rightPreview.scale,
                      params->config.singleLaneHeadingConfidence));
        CHECK(rightPreview.reason == LateralScaleReason::SINGLE_PREVIEW);

        // RIGHT_SINGLE 左弯恢复：near=-5 -> scale>0.45。
        const auto rightRecovery5 = singleLaneLateralScale(
            LaneRecoveryMode::RIGHT_SINGLE, true, true, -5.0f, -50.0f, 1.0f,
            params->config);
        const float expected5 = 0.45f + 0.55f * (5.0f / 30.0f);
        CHECK(closeTo(rightRecovery5.scale, expected5));
        CHECK(rightRecovery5.reason == LateralScaleReason::SINGLE_RECOVERY);

        // near=-15 -> scale 进一步增大。
        const auto rightRecovery15 = singleLaneLateralScale(
            LaneRecoveryMode::RIGHT_SINGLE, true, true, -15.0f, -70.0f, 1.0f,
            params->config);
        CHECK(closeTo(rightRecovery15.scale, 0.45f + 0.55f * 0.5f));
        CHECK(rightRecovery15.scale > rightRecovery5.scale);

        // near<=-30 -> full。
        const auto rightFull = singleLaneLateralScale(
            LaneRecoveryMode::RIGHT_SINGLE, true, true, -30.0f, -80.0f, 1.0f,
            params->config);
        CHECK(closeTo(rightFull.scale, 1.0f));

        // LEFT_SINGLE 右弯镜像：scale 与 reason 完全一致。
        const auto leftPreview = singleLaneLateralScale(
            LaneRecoveryMode::LEFT_SINGLE, true, true, -5.0f, 40.0f, 1.0f,
            params->config);
        CHECK(closeTo(leftPreview.scale, rightPreview.scale));
        CHECK(leftPreview.reason == rightPreview.reason);
        const auto leftRecovery5 = singleLaneLateralScale(
            LaneRecoveryMode::LEFT_SINGLE, true, true, 5.0f, 50.0f, 1.0f,
            params->config);
        CHECK(closeTo(leftRecovery5.scale, rightRecovery5.scale));
        CHECK(leftRecovery5.reason == rightRecovery5.reason);

        // far invalid + headingConfidence=0 + near valid -> full lateral。
        const auto noHeading = singleLaneLateralScale(
            LaneRecoveryMode::RIGHT_SINGLE, true, false, -20.0f, 0.0f,
            0.0f, params->config);
        CHECK(closeTo(noHeading.scale, 1.0f));
        CHECK(noHeading.reason == LateralScaleReason::SINGLE_NO_HEADING);

        // near invalid -> 不允许异常 full-scale（保持 base）。
        const auto nearInvalid = singleLaneLateralScale(
            LaneRecoveryMode::RIGHT_SINGLE, false, false, 0.0f, 0.0f,
            0.0f, params->config);
        CHECK(closeTo(nearInvalid.scale,
                      params->config.singleLaneHeadingConfidence));
        CHECK(nearInvalid.scale < 1.0f);
    }
    // 2. recovery damping 开关：仅真实双边模式启用 0.25。
    {
        CHECK(recoveryDampingEnabledForMode(LaneRecoveryMode::STRICT_DUAL));
        CHECK(recoveryDampingEnabledForMode(LaneRecoveryMode::RELAXED_DUAL));
        CHECK(!recoveryDampingEnabledForMode(LaneRecoveryMode::WEAK_HYBRID));
        CHECK(!recoveryDampingEnabledForMode(LaneRecoveryMode::LEFT_SINGLE));
        CHECK(!recoveryDampingEnabledForMode(LaneRecoveryMode::RIGHT_SINGLE));
    }
    // 2b. WEAK_HYBRID 冲突仲裁纯函数（左右镜像）。
    {
        auto params = makeTestParams();
        const auto conflictLeft = weakHybridConflictLateralScale(
            LaneRecoveryMode::WEAK_HYBRID, true, true, 48.0f, -13.0f,
            0.35f, 64.0f, -23.0f, params->config);
        CHECK(conflictLeft.reason ==
              LateralScaleReason::WEAK_HYBRID_CONFLICT);
        CHECK(closeTo(conflictLeft.scale, 0.35f));
        const auto conflictRight = weakHybridConflictLateralScale(
            LaneRecoveryMode::WEAK_HYBRID, true, true, -48.0f, 13.0f,
            0.35f, -64.0f, 23.0f, params->config);
        CHECK(conflictRight.reason ==
              LateralScaleReason::WEAK_HYBRID_CONFLICT);
        CHECK(closeTo(conflictRight.scale, conflictLeft.scale));
        // near/far 同侧：不冲突。
        const auto sameSide = weakHybridConflictLateralScale(
            LaneRecoveryMode::WEAK_HYBRID, true, true, -10.0f, -50.0f,
            0.4f, -20.0f, -30.0f, params->config);
        CHECK(sameSide.reason == LateralScaleReason::FULL_DUAL);
        CHECK(closeTo(sameSide.scale, 1.0f));
        // heading 无效：不冲突。
        const auto noHeading = weakHybridConflictLateralScale(
            LaneRecoveryMode::WEAK_HYBRID, true, true, 48.0f, -13.0f,
            0.0f, 64.0f, 0.0f, params->config);
        CHECK(noHeading.reason == LateralScaleReason::FULL_DUAL);
        CHECK(closeTo(noHeading.scale, 1.0f));
        // far invalid：不能错误削弱 lateral。
        const auto farInvalid = weakHybridConflictLateralScale(
            LaneRecoveryMode::WEAK_HYBRID, true, false, 48.0f, 0.0f,
            0.4f, 64.0f, -23.0f, params->config);
        CHECK(farInvalid.reason == LateralScaleReason::FULL_DUAL);
        CHECK(closeTo(farInvalid.scale, 1.0f));
        // lateral/heading 同方向：不冲突。
        const auto sameDirection = weakHybridConflictLateralScale(
            LaneRecoveryMode::WEAK_HYBRID, true, true, 48.0f, -13.0f,
            0.4f, -64.0f, -23.0f, params->config);
        CHECK(sameDirection.reason == LateralScaleReason::FULL_DUAL);
        CHECK(closeTo(sameDirection.scale, 1.0f));
        // 非 WEAK_HYBRID 不受影响。
        const auto dualMode = weakHybridConflictLateralScale(
            LaneRecoveryMode::STRICT_DUAL, true, true, 48.0f, -13.0f,
            1.0f, 64.0f, -23.0f, params->config);
        CHECK(dualMode.reason == LateralScaleReason::FULL_DUAL);
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
    // 5b. 动态恢复 scale 应用于 Motion：lateralApplied = raw × scale，
    //     pwmDiff 左右镜像。
    {
        auto params = makeTestParams();
        const auto recoveryScale = singleLaneLateralScale(
            LaneRecoveryMode::RIGHT_SINGLE, true, true, -15.0f, -70.0f,
            1.0f, params->config);
        params->ctrl.center = 145; // error=-15（RIGHT_SINGLE 左弯恢复）
        params->ctrl.laneHeadingCorrection = -31.5f;
        params->ctrl.laneLateralScale = recoveryScale.scale;
        Motion motion;
        motion.reset();
        motion.poseControl(params, 0.03f);
        const float raw = -15.0f * (15.0f * params->config.runP2 +
                                    params->config.runP1);
        CHECK(closeTo(params->ctrl.lateralRaw, raw, 1e-2f));
        CHECK(closeTo(params->ctrl.lateralApplied,
                      raw * recoveryScale.scale, 1e-2f));
        CHECK(closeTo(params->ctrl.headingApplied, -31.5f));
        const int rightPwmDiff = params->ctrl.pwmDiff;

        params->ctrl.center = 175; // error=+15（LEFT_SINGLE 右弯镜像）
        params->ctrl.laneHeadingCorrection = 31.5f;
        params->ctrl.laneLateralScale = recoveryScale.scale;
        Motion mirrorMotion;
        mirrorMotion.reset();
        mirrorMotion.poseControl(params, 0.03f);
        CHECK(closeTo(params->ctrl.lateralApplied,
                      -raw * recoveryScale.scale, 1e-2f));
        CHECK(params->ctrl.pwmDiff == -rightPwmDiff);
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
    // 11. WEAK_HYBRID 冲突时 Motion 最终转向明显向左（不被 lateral 抵消）。
    {
        auto params = makeTestParams();
        params->ctrl.center = 187; // error=+27 -> lateralRaw≈63.8
        params->ctrl.laneHeadingCorrection = -23.0f;
        params->ctrl.laneLateralScale = 1.0f; // Center 基础值
        params->ctrl.laneLateralScaleReason = 0;
        params->ctrl.laneRecoveryMode =
            static_cast<int>(LaneRecoveryMode::WEAK_HYBRID);
        params->ctrl.laneNearValid = true;
        params->ctrl.laneFarValid = true;
        params->ctrl.laneNearError = 48.0f;
        params->ctrl.laneFarError = -13.0f;
        params->ctrl.laneHeadingConfidence = 0.35f;
        Motion motion;
        motion.reset();
        motion.poseControl(params, 0.03f);
        CHECK(closeTo(params->ctrl.lateralRaw, 63.8f, 1e-1f));
        CHECK(closeTo(params->ctrl.laneLateralScale, 0.35f));
        CHECK(params->ctrl.laneLateralScaleReason ==
              static_cast<int>(LateralScaleReason::WEAK_HYBRID_CONFLICT));
        const int conflictPwmDiff = params->ctrl.pwmDiff;
        CHECK(conflictPwmDiff < 0); // 提前左转
        CHECK(params->ctrl.servo > 1500);

        // 对照：heading 无效（不触发冲突）-> lateral 100% 仍右转。
        params->ctrl.laneHeadingConfidence = 0.0f;
        params->ctrl.laneLateralScale = 1.0f; // 恢复 Center 基础值
        params->ctrl.laneLateralScaleReason = 0;
        Motion noConflictMotion;
        noConflictMotion.reset();
        noConflictMotion.poseControl(params, 0.03f);
        CHECK(params->ctrl.pwmDiff > 0);
        CHECK(params->ctrl.servo < 1500);
        CHECK(conflictPwmDiff < params->ctrl.pwmDiff);
    }
    // 12. INVALID/reset 后诊断字段清零，不残留上一有效帧。
    {
        auto params = makeTestParams();
        params->ctrl.lateralRaw = 123.0f;
        params->ctrl.lateralApplied = 50.0f;
        params->ctrl.headingApplied = -20.0f;
        params->ctrl.pwmDiff = 99;
        params->ctrl.laneLateralScale = 0.45f;
        params->ctrl.laneLateralScaleReason = 2;
        params->ctrl.laneNearError = 48.0f;
        params->ctrl.laneRecoveryMode =
            static_cast<int>(LaneRecoveryMode::WEAK_HYBRID);
        Center center;
        center.fitting(params); // 空轨道 -> resetControlGeometry
        CHECK(params->ctrl.lateralRaw == 0.0f);
        CHECK(params->ctrl.lateralApplied == 0.0f);
        CHECK(params->ctrl.headingApplied == 0.0f);
        CHECK(params->ctrl.pwmDiff == 0);
        CHECK(params->ctrl.laneLateralScale == 1.0f);
        CHECK(params->ctrl.laneLateralScaleReason == 0);
        CHECK(params->ctrl.laneNearError == 0.0f);
        CHECK(params->ctrl.laneRecoveryMode ==
              static_cast<int>(LaneRecoveryMode::INVALID));
    }
    return 0;
}
