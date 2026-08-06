#include "icar.hpp"

void Icar::applyFinalStopArbitration(FrameCycle &frame)
{
        // Reassert after all recovery/control logic so the latch cannot be cleared this frame.
        if (frame.emergencyStopRequested)
        {
            params->setStopReason(control_algorithms::StopReason::EMERGENCY, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }
        if (!frame.startupGateReleased)
        {
            params->setStopReason(control_algorithms::StopReason::STARTUP, true);
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }

        params->ctrl.stop = params->mustStop() || !frame.cameraReady;
        const bool geometrySafetyHold = params->stopReasons.hasOnly(
            control_algorithms::StopReason::LANE,
            control_algorithms::StopReason::PLANNER);
        FinalCommand command = resolveFinalCommand(
            {params->ctrl.speed, params->ctrl.servo},
            {params->mustStop() && !geometrySafetyHold,
             frame.emergencyStopRequested,
             frame.cameraReady, frame.startupGateReleased},
            PWMSERVOMID);
        if (geometrySafetyHold)
            command.speed = 0.0f;
        params->ctrl.speed = command.speed;
        params->ctrl.servo = static_cast<uint16_t>(command.servo);
        previousFinalServo = params->ctrl.servo;
}
