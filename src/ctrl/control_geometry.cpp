#include "ctrl/center.hpp"

void Center::resetControlGeometry(Params &params)
{
    sigmaCenter = 0;
    params.ctrl.center = COLSIMAGE / 2;
    nearCenter = COLSIMAGE / 2;
    farCenter = COLSIMAGE / 2;
    headingError = 0.0f;
    headingCorrection = 0.0f;
    headingConfidence = 0.0f;
    params.ctrl.laneHeadingCorrection = 0.0f;
    nearCenterSamples = 0;
    farCenterSamples = 0;
    nearCenterValid = false;
    farCenterValid = false;
    singleSide = 0;
    rawCenterJump = 0;
    appliedCenterStep = 0;
    usableCenterRows = 0;
    recoveryMode = LaneRecoveryMode::INVALID;
    rejectedPathSource = PathSource::NONE;
    plannedValidation = PlannedPathValidation{};
    geometry.reset();
    geometry.updated = true;
}

void Center::applyControlGeometry(Params &params, bool plannedPath,
                                  bool perceptionPath, PathSource pathSource)
{
    geometry.source = plannedPath ? ControlGeometrySource::PLANNED :
        (perceptionPath ? ControlGeometrySource::PERCEPTION :
                          ControlGeometrySource::NONE);
    geometry.pathSource = plannedPath ? pathSource : PathSource::NONE;
    geometry.centerLine = params.ctrl.centerEdge;
    geometry.valid = controlValid;
}
