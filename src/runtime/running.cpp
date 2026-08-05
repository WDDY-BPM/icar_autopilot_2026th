#include "icar.hpp"

void Icar::running()
{
    FrameCycle frame;
    frame.startupGateReleased = startupGateState == StartupGateState::RELEASED;
    acquireFrame(frame);
    if (frame.frameAvailable)
    {
        preprocessFrame(frame);
        params->advancePathFrame();
        lastTelemetryFrameId = frame.frameId;
        lastTelemetryTimestampMs = frame.timestampMs;
        consumeAiSnapshot(frame);
        updateSafetyState(frame);
        runStateMachines(frame);
        calculateControl(frame);
    }
    else
    {
        frame.frameId = lastTelemetryFrameId;
        frame.timestampMs = lastTelemetryTimestampMs;
        updateSafetyState(frame);
    }
    applyFinalStopArbitration(frame);
    publishTelemetry(frame);
    sendVehicleCommand(frame);
    if (frame.exitRequested)
        requestShutdown();
}

void Icar::requestShutdown()
{
    shutdownRequested = true;
    shuttingDown = true;
    cvImg.notify_all();
}
