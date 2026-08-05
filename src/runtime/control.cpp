#include "icar.hpp"

void Icar::calculateControl(FrameCycle &frame)
{
        //[06] Calculate the lane control center in autonomous mode.
        frame.centerUpdated = false;
        if (frame.cameraReady && frame.startupGateReleased && !frame.emergencyStopRequested &&
            !params->manualTakeover && params->autoRecoveryFrames <= 0 && !frame.aiStale)
        {
            center->fitting(params);
            frame.centerUpdated = true;
            const int laneUnconfirmedFrames = control_algorithms::updateLaneUnconfirmed(
                laneUnconfirmedState, center->controlValid, 5);
            const bool safetyLaneMode = params->mode == FsmMode::NORMAL ||
                                        params->mode == FsmMode::CROSS ||
                                        params->mode == FsmMode::STOP ||
                                        params->mode == FsmMode::SLOW ||
                                        params->mode == FsmMode::STATION;
            params->laneSafetyStop = control_algorithms::updateLaneSafetyStop(
                params->laneSafetyStop, safetyLaneMode, center->controlValid,
                center->laneInvalidFrames, center->laneRecoveryFrames, 7, 5,
                laneUnconfirmedFrames, 30);
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
        const bool laneHold = params->laneSafetyStop;
        static bool laneHoldWasActive = false;
        if (automaticControlActive && !laneHold)
        {
            motion->poseControl(params, steeringDt);
            motion->speedControl(params);

            const bool strictLaneMode = params->mode == FsmMode::NORMAL ||
                                        params->mode == FsmMode::CROSS ||
                                        params->mode == FsmMode::STOP ||
                                        params->mode == FsmMode::SLOW ||
                                        params->mode == FsmMode::STATION;
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

            if (strictLaneMode && laneUnconfirmedState.frames > 0)
            {
                params->ctrl.speed = std::min(params->ctrl.speed, 0.10f);
                if (!center->controlValid)
                    params->ctrl.servo = motion->syncServoCommand(previousFinalServo);
            }
        }
        else if (automaticControlActive && laneHold)
        {
            if (!laneHoldWasActive)
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
        laneHoldWasActive = automaticControlActive && laneHold;
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
