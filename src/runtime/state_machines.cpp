#include "icar.hpp"

void Icar::runStateMachines(FrameCycle &frame)
{
        if (frame.cameraReady && frame.startupGateReleased &&
            !frame.emergencyStopRequested &&
            params->autoRecoveryFrames <= 0 && !params->laneSafetyStop &&
            (frame.manualBeforeFsm || !frame.aiStale))
            runFsm(frame.binary);

        params->dropPathOverrideIfDisallowed(params->mode);

        // 同步手动接管状态（runFsm中endManualTakeover可能改变了状态，但params->manualTakeover未更新）
        params->manualTakeover = fsmFactory.busy->isInManualTakeover();
        if (params->manualTakeover != frame.manualBeforeFsm)
            frame.aiStale = updateAiSafety(!params->manualTakeover, false);

        // A remote STOP is latched and has priority in AUTO and MANUAL modes.
        if (frame.emergencyStopRequested)
        {
            params->setStopReason(control_algorithms::StopReason::EMERGENCY, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }
}
