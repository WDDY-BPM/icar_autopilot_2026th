#include "icar.hpp"

void Icar::running()
{
    FrameCycle frame;
    if (!acquireFrame(frame))
        return;
    preprocessFrame(frame);
    consumeAiSnapshot(frame);
    updateSafetyState(frame);
    runStateMachines(frame);
    calculateControl(frame);
    applyFinalStopArbitration(frame);
    publishTelemetry(frame);
    sendVehicleCommand(frame);
}