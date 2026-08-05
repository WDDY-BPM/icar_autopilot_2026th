#include "icar.hpp"

bool Icar::updateAiSafety(bool automaticMode, bool successfulFreshResult,
                        std::int64_t successfulResultMs)
    {
        const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const bool wasStale = aiFreshness.stale;
        const bool stale = control_algorithms::updateAiFreshness(
            aiFreshness, automaticMode, successfulFreshResult, nowMs,
            control_algorithms::AI_STALE_TIMEOUT_MS,
            control_algorithms::AI_RECOVERY_FRESH_RESULTS, successfulResultMs);
        params->setStopReason(control_algorithms::StopReason::AI_STALE, stale);
        if (automaticMode && stale != wasStale)
            std::cout << (stale ? "[AI] Results stale; AI_STALE stop active."
                                : "[AI] Fresh results recovered; AI_STALE stop cleared.")
                      << std::endl;
        return stale;
    }

bool Icar::updateStartupGate(bool receivedNewAiResult)
    {
        if (!startupEnvironmentChecked)
        {
            startupEnvironmentChecked = true;
            const char *preconfirmed = std::getenv("ICAR_START_CONE_PRECONFIRMED");
            if (params->config.requireStartCone && preconfirmed &&
                std::string(preconfirmed) == "1")
            {
                startupGateState = StartupGateState::WAIT_FOR_REMOVAL;
                startupConeSeenCount = 3;
                startupConeMissingCount = 0;
                std::cout << "[Startup] Cone preconfirmed by launcher. "
                             "Waiting for removal." << std::endl;
            }
        }
        if (startupGateState == StartupGateState::RELEASED)
            return true;

        bool coneDetected = startupConeDetected;
        if (receivedNewAiResult)
        {
            coneDetected = false;
            for (const auto &result : params->results)
            {
                if (result.type == LABEL_CONE && result.width >= 10 && result.height >= 10)
                {
                    coneDetected = true;
                    break;
                }
            }
            startupConeDetected = coneDetected;
        }

        const auto &laneQuality = params->track->quality;
        // At a curved start line, temporal center jump and perspective lane
        // width variation can legitimately exceed the straight-road quality
        // thresholds.  Requiring those metrics here could keep the startup
        // gate closed forever even though both physical lane edges are sound.
        // The normal controller performs its own recovery checks after release.
        const bool laneValid = laneQuality.leftReliable &&
            laneQuality.rightReliable && laneQuality.coversBottom &&
            laneQuality.commonRows >= 20;
        startupLaneValidCount = laneValid ? startupLaneValidCount + 1 : 0;

        if (!params->config.requireStartCone)
        {
            if (startupLaneValidCount >= params->config.startupStableFrames)
            {
                startupGateState = StartupGateState::RELEASED;
                params->ctrl.countAcc = 0;
                params->ctrl.startupSteeringCount = 0;
                params->setStopReason(control_algorithms::StopReason::STARTUP, false);
                std::cout << "[Startup] Cone gate disabled; stable lane confirmed. AUTO released." << std::endl;
                return true;
            }
            params->setStopReason(control_algorithms::StopReason::STARTUP, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
            return false;
        }

        if (startupGateState == StartupGateState::WAIT_FOR_CONE)
        {
            if (receivedNewAiResult)
                startupConeSeenCount = coneDetected ? startupConeSeenCount + 1 : 0;

            if (startupConeSeenCount >= 3)
            {
                startupGateState = StartupGateState::WAIT_FOR_REMOVAL;
                startupConeMissingCount = 0;
                std::cout << "[Startup] Cone confirmed. Remove it to start." << std::endl;
            }
        }
        else if (receivedNewAiResult)
        {
            startupConeMissingCount = coneDetected ? 0 : startupConeMissingCount + 1;
            if (startupConeMissingCount >= 5 && startupLaneValidCount >= params->config.startupStableFrames)
            {
                startupGateState = StartupGateState::RELEASED;
                params->ctrl.countAcc = 0;
                params->ctrl.startupSteeringCount = 0;
                params->setStopReason(control_algorithms::StopReason::STARTUP, false);
                std::cout << "[Startup] Cone removed and lane stable. AUTO released." << std::endl;
                return true;
            }
        }

        params->setStopReason(control_algorithms::StopReason::STARTUP, true);
        params->ctrl.speed = 0.0f;
        params->ctrl.servo = PWMSERVOMID;
        if (++startupDiagnosticFrames % 30 == 0)
        {
            const char *state = startupGateState == StartupGateState::WAIT_FOR_CONE
                ? "WAIT_FOR_CONE" : "WAIT_FOR_REMOVAL";
            std::cout << "[Startup] state=" << state
                      << " coneDetected=" << coneDetected
                      << " coneSeen=" << startupConeSeenCount
                      << " coneMissing=" << startupConeMissingCount
                      << " leftReliable=" << laneQuality.leftReliable
                      << " rightReliable=" << laneQuality.rightReliable
                      << " coversBottom=" << laneQuality.coversBottom
                      << " commonRows=" << laneQuality.commonRows
                      << " laneValid=" << laneValid
                      << " laneFrames=" << startupLaneValidCount
                      << " confidence=" << laneQuality.confidence
                      << " centerJump=" << laneQuality.centerJump
                      << " widthVariation=" << laneQuality.widthVariation
                      << std::endl;
        }
        return false;
    }
