#include "icar.hpp"
#include "runtime/control_decision.hpp"

void Icar::calculateControl(FrameCycle &frame)
{
        //[06] Calculate the lane control center in autonomous mode.
        frame.centerUpdated = false;
        center->geometry.reset();
        params->reconcilePlannerSafetyWithMode();
        if (frame.cameraReady && frame.startupGateReleased && !frame.emergencyStopRequested &&
            !params->manualTakeover && params->autoRecoveryFrames <= 0 && !frame.aiStale)
        {
            center->fitting(params);
            frame.centerUpdated = true;
            if (center->rejectedPathSource != PathSource::NONE)
                params->plannerSafety.reject(center->rejectedPathSource);
            if (params->plannerSafety.latched)
            {
                const bool matchingValidPlan = center->geometry.updated &&
                    center->geometry.source == ControlGeometrySource::PLANNED &&
                    center->geometry.pathSource ==
                        params->plannerSafety.rejectedSource &&
                    center->geometry.valid;
                params->plannerSafety.observeFrame(matchingValidPlan);
            }
            params->setStopReason(control_algorithms::StopReason::PLANNER,
                                  params->plannerSafety.latched);

            const bool perceptionGeometry =
                center->geometry.source == ControlGeometrySource::PERCEPTION;
            if (perceptionGeometry)
            {
                const int laneUnconfirmedFrames =
                    control_algorithms::updateLaneUnconfirmed(
                        laneUnconfirmedState, center->controlValid, 5);
                params->laneSafetyStop = control_algorithms::updateLaneSafetyStop(
                    params->laneSafetyStop, true, center->controlValid,
                    center->laneInvalidFrames, center->laneRecoveryFrames, 7, 5,
                    laneUnconfirmedFrames, 30);
            }
            else
            {
                laneUnconfirmedState = control_algorithms::LaneUnconfirmedState{};
                params->laneSafetyStop = false;
            }
            params->setStopReason(control_algorithms::StopReason::LANE,
                                  params->laneSafetyStop);
        }

        params->ctrl.stop = params->mustStop();

        //[07] 车辆运动控制（仅手动接管时跳过）
        static auto lastSteeringUpdate = std::chrono::steady_clock::now();
        static bool steeringClockInitialized = false;
        const auto steeringNow = std::chrono::steady_clock::now();
        float steeringDt = 1.0f / 30.0f;
        if (steeringClockInitialized)
            steeringDt = std::chrono::duration<float>(steeringNow - lastSteeringUpdate).count();
        lastSteeringUpdate = steeringNow;
        steeringClockInitialized = true;

        const bool automaticControlActive =
            frame.cameraReady && frame.startupGateReleased && !frame.emergencyStopRequested &&
            !params->manualTakeover && params->autoRecoveryFrames <= 0 && !frame.aiStale;
        center->geometry.updated = frame.centerUpdated;
        const RuntimeControlDecision controlDecision = evaluateRuntimeControl(
            center->geometry, params->stopReasons, automaticControlActive);
        static bool automaticHoldWasActive = false;
        if (controlDecision.allowMotion)
        {
            motion->poseControl(params, steeringDt);
            motion->speedControl(params);

            const bool strictLaneMode = center->geometry.source ==
                ControlGeometrySource::PERCEPTION;
            const bool lowConfidenceLane =
                center->recoveryMode == LaneRecoveryMode::WEAK_HYBRID ||
                center->recoveryMode == LaneRecoveryMode::LEFT_SINGLE ||
                center->recoveryMode == LaneRecoveryMode::RIGHT_SINGLE;
            const bool strictDualLane =
                center->recoveryMode == LaneRecoveryMode::STRICT_DUAL;
            const bool retainedSingleLaneLimit =
                control_algorithms::updateSingleLaneSpeedLimit(
                    singleLaneSpeedLimit, strictLaneMode, center->controlValid,
                    strictDualLane, strictDualLane, 5, lowConfidenceLane);
            if (retainedSingleLaneLimit)
                params->ctrl.speed = std::min(params->ctrl.speed, 0.15f);
            else if (center->recoveryMode == LaneRecoveryMode::RELAXED_DUAL)
                params->ctrl.speed = std::min(params->ctrl.speed,
                    std::min(params->config.velCurve, 0.20f));
            if (params->yforkPhase == YforkRuntimePhase::PERCEPTION_RECOVERY)
                params->ctrl.speed = std::min(params->ctrl.speed,
                                              params->config.velYfork);

            if (strictLaneMode && laneUnconfirmedState.frames > 0)
            {
                params->ctrl.speed = std::min(params->ctrl.speed, 0.10f);
                if (!center->controlValid)
                    params->ctrl.servo = motion->syncServoCommand(previousFinalServo);
            }
        }
        else if (controlDecision.centerSteering)
        {
            if (!automaticHoldWasActive)
                params->ctrl.countAcc = 0;
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = motion->limitServoCommand(
                PWMSERVOMID, steeringDt, 250.0f);
            if (std::abs(static_cast<int>(params->ctrl.servo) - PWMSERVOMID) <= 3)
                motion->resetControl();
        }
        else if (params->manualTakeover && !frame.emergencyStopRequested)
        {
            params->laneSafetyStop = false;
            params->setStopReason(control_algorithms::StopReason::LANE, false);
            params->plannerSafety.clear();
            params->setStopReason(control_algorithms::StopReason::PLANNER, false);
            motion->resetControl();
            params->ctrl.servo = motion->limitServoCommand(
                params->ctrl.servo, steeringDt, params->config.servoRate);
        }
        else
        {
            // Startup gating and emergency/recovery stops must return to center
            // immediately instead of passing through the normal slew limiter.
            motion->reset();
        }
        automaticHoldWasActive = controlDecision.centerSteering;
        if (!frame.emergencyStopRequested && params->autoRecoveryFrames > 0)
        {
            params->setStopReason(control_algorithms::StopReason::STARTUP, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
            params->autoRecoveryFrames--;
            if (params->autoRecoveryFrames == 0)
                params->setStopReason(control_algorithms::StopReason::STARTUP, false);
        }
}
