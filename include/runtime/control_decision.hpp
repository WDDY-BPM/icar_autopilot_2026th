#pragma once

#include "ctrl/control_geometry.hpp"
#include "ctrl/stop_reasons.hpp"

struct RuntimeControlDecision
{
    bool allowMotion{false};
    bool centerSteering{false};
};

inline RuntimeControlDecision evaluateRuntimeControl(
    const ControlGeometry &geometry,
    const control_algorithms::StopReasonState &stopReasons,
    bool automaticEnabled)
{
    const bool usableGeometry = geometry.updated && geometry.valid &&
        geometry.source != ControlGeometrySource::NONE &&
        !geometry.centerLine.empty();
    return {automaticEnabled && usableGeometry && !stopReasons.mustStop(),
            automaticEnabled && (!usableGeometry || stopReasons.mustStop())};
}
