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

        params->ctrl.stop = params->mustStop();
        if (params->ctrl.stop)
        {
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }
        previousFinalServo = params->ctrl.servo;
}
