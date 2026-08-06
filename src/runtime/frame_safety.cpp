#include "icar.hpp"

void Icar::updateSafetyState(FrameCycle &frame)
{
        //[05] 有限状态机任务执行。锁存急停时不得推进任何有状态 FSM。
        frame.emergencyStopRequested = fsmFactory.manual->isEmergencyStopRequested();
        params->setStopReason(control_algorithms::StopReason::EMERGENCY,
                              frame.emergencyStopRequested);
        if (frame.emergencyStopRequested && !emergencyStopWasActive)
        {
            emergencyStopWasActive = true;
        }
        else if (!frame.emergencyStopRequested && emergencyStopWasActive)
        {
            // All FSM state and parking trajectory history were frozen with the vehicle.
            // Keep them intact, gather fresh lane data while stopped, then continue.
            params->autoRecoveryFrames = 15;
            emergencyStopWasActive = false;
        }
}
