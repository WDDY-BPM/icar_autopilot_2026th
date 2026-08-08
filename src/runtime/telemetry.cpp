#include "icar.hpp"
#include <fstream>
#include <iomanip>
#include <string>

namespace
{
std::ofstream &driveLogFile()
{
    // Run ./icar from build/. A fresh file is created for each program start.
    static std::ofstream file("drive_log.csv",
                              std::ios::out | std::ios::trunc);
    static bool headerWritten = false;

    if (!headerWritten && file.is_open())
    {
        file
            << "timestamp_ms,"
            << "frame_id,"
            << "mode,"
            << "speed,"
            << "servo,"
            << "servo_offset,"
            << "center,"
            << "center_error,"
            << "lane_mode,"
            << "near_error,"
            << "far_error,"
            << "heading_rad,"
            << "heading_deg,"
            << "heading_correction,"
            << "heading_confidence,"
            << "opposite_side_recovery,"
            << "recovery_damping,"
            << "lane_confidence,"
            << "common_rows,"
            << "left_count,"
            << "right_count,"
            << "left_reliable,"
            << "right_reliable,"
            << "left_single_usable,"
            << "right_single_usable,"
            << "left_border_ratio,"
            << "right_border_ratio,"
            << "lane_width_ready,"
            << "raw_center_jump,"
            << "applied_center_step,"
            << "control_valid,"
            << "path_active,"
            << "path_source,"
            << "path_age_ms,"
            << "path_remaining_ms,"
            << "path_speed_limit,"
            << "must_stop,"
            << "stop_reasons,"
            << "detections"
            << '\n';
        headerWritten = true;
    }

    return file;
}

std::string csvEscape(const std::string &value)
{
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (char ch : value)
    {
        if (ch == '"')
            escaped += "\"\"";
        else if (ch == '\r' || ch == '\n')
            escaped.push_back(' ');
        else
            escaped.push_back(ch);
    }
    escaped.push_back('"');
    return escaped;
}

nlohmann::json detectionSummary(const std::vector<PredictResult> &results)
{
    nlohmann::json detections = nlohmann::json::array();
    for (const auto &result : results)
    {
        detections.push_back({
            {"type", result.type},
            {"label", result.label},
            {"score", result.score},
            {"x", result.x},
            {"y", result.y},
            {"w", result.width},
            {"h", result.height}
        });
    }
    return detections;
}
} // namespace


void Icar::publishTelemetry(const FrameCycle &frame)
{
        // Publish the final command after automatic/manual limiting and all
        // emergency/startup overrides, so telemetry is not one frame stale.
        fsmFactory.manual->updateVehicleState(
            params->ctrl.speed, params->ctrl.servo, params->manualTakeover);

        // Persistent drive diagnostics at 10 Hz. This is intentionally outside
        // the remote-overlay connection check, so the file is recorded even
        // when no manual/telemetry client is connected.
        static auto lastDriveLogAt = std::chrono::steady_clock::time_point{};
        static int driveLogFlushCounter = 0;
        const auto driveLogNow = std::chrono::steady_clock::now();

        if (frame.frameId != 0 &&
            driveLogNow - lastDriveLogAt >= std::chrono::milliseconds(100))
        {
            lastDriveLogAt = driveLogNow;
            auto &log = driveLogFile();

            if (log.is_open())
            {
                const int64_t timestampMs =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        driveLogNow.time_since_epoch()).count();

                constexpr double kRadToDeg = 57.29577951308232;
                const int nearError = center->nearCenterValid
                    ? center->nearCenter - COLSIMAGE / 2 : 0;
                const int farError = center->farCenterValid
                    ? center->farCenter - COLSIMAGE / 2 : 0;

                const std::string detections =
                    detectionSummary(params->results).dump();

                log
                    << timestampMs << ','
                    << frame.frameId << ','
                    << static_cast<int>(params->mode) << ','
                    << std::fixed << std::setprecision(3)
                    << params->ctrl.speed << ','
                    << params->ctrl.servo << ','
                    << (static_cast<int>(params->ctrl.servo) - PWMSERVOMID) << ','
                    << params->ctrl.center << ','
                    << (params->ctrl.center - COLSIMAGE / 2) << ','
                    << laneRecoveryModeName(center->recoveryMode) << ','
                    << nearError << ','
                    << farError << ','
                    << std::setprecision(6)
                    << center->headingError << ','
                    << std::setprecision(2)
                    << (static_cast<double>(center->headingError) * kRadToDeg) << ','
                    << center->headingCorrection << ','
                    << center->headingConfidence << ','
                    << center->oppositeSideRecovery << ','
                    << center->recoveryDamping << ','
                    << params->track->quality.confidence << ','
                    << params->track->quality.commonRows << ','
                    << params->track->pointsEdgeLeft.size() << ','
                    << params->track->pointsEdgeRight.size() << ','
                    << params->track->quality.leftReliable << ','
                    << params->track->quality.rightReliable << ','
                    << params->track->quality.leftSingleUsable << ','
                    << params->track->quality.rightSingleUsable << ','
                    << params->track->quality.leftBorderRatio << ','
                    << params->track->quality.rightBorderRatio << ','
                    << center->laneWidthProfileReady() << ','
                    << center->rawCenterJump << ','
                    << center->appliedCenterStep << ','
                    << center->controlValid << ','
                    << params->pathOverride.active() << ','
                    << pathSourceName(params->pathOverride.source) << ','
                    << params->pathOverride.ageMs() << ','
                    << params->pathOverride.remainingMs() << ','
                    << params->pathOverride.speedLimit << ','
                    << params->mustStop() << ','
                    << csvEscape(params->stopReasonString()) << ','
                    << csvEscape(detections)
                    << '\n';

                // Flush about once per second instead of on every row.
                ++driveLogFlushCounter;
                if (driveLogFlushCounter >= 10)
                {
                    log.flush();
                    driveLogFlushCounter = 0;
                }
            }
        }
        // Construct and publish overlays at no more than 12.5 Hz. This avoids
        // allocating and serializing JSON on every 30 Hz control iteration.
        const auto overlayNow = std::chrono::steady_clock::now();
        if (fsmFactory.manual->isConnected() && frame.frameId != 0 &&
            overlayNow - lastOverlayBuilt >= std::chrono::milliseconds(80))
        {
            lastOverlayBuilt = overlayNow;
            const bool leftOverlayValid = frame.lanesUpdated &&
                                          (params->track->quality.leftReliable ||
                                           center->singleSide == -1 ||
                                           center->recoveryMode == LaneRecoveryMode::WEAK_HYBRID) &&
                                          !params->manualTakeover;
            const bool rightOverlayValid = frame.lanesUpdated &&
                                           (params->track->quality.rightReliable ||
                                            center->singleSide == 1 ||
                                            center->recoveryMode == LaneRecoveryMode::WEAK_HYBRID) &&
                                           !params->manualTakeover;
            const bool lanesValid = leftOverlayValid || rightOverlayValid;
            const bool centerValid = frame.centerUpdated &&
                                     center->controlValid &&
                                     params->ctrl.centerEdge.size() >= 12 &&
                                     !params->manualTakeover;
            nlohmann::json overlay;
            overlay["frame_id"] = frame.frameId;
            overlay["frame_timestamp_ms"] = frame.timestampMs;
            overlay["timestamp_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            overlay["mode"] = static_cast<int>(params->mode);
            overlay["speed"] = params->ctrl.speed;
            overlay["steering"] = params->ctrl.servo;
            overlay["center"] = params->ctrl.center;
            overlay["center_error"] = params->ctrl.center - COLSIMAGE / 2;
            overlay["recovery_mode"] =
                laneRecoveryModeName(center->recoveryMode);
            overlay["ctrl_stop"] = params->ctrl.stop;
            overlay["must_stop"] = params->mustStop();
            overlay["stop_reasons"] = params->stopReasonString();
            overlay["ai_stale"] = params->hasStopReason(
                control_algorithms::StopReason::AI_STALE);
            overlay["camera_stop"] = params->hasStopReason(
                control_algorithms::StopReason::CAMERA);
            overlay["camera_ready"] = frame.cameraReady;
            overlay["camera_timed_out"] = frame.cameraTimedOut;
            overlay["path_override_active"] = params->pathOverride.active();
            overlay["path_source"] = pathSourceName(params->pathOverride.source);
            overlay["path_ttl_frames"] = params->pathOverride.ttlFrames;
            overlay["path_generated_frame_id"] = params->pathOverride.generatedFrameId;
            overlay["path_age_ms"] = params->pathOverride.ageMs();
            overlay["path_validity_mode"] =
                params->pathOverride.freshnessMode == PathFreshnessMode::TIME_TTL
                    ? "TIME_TTL" : "CONTROL_FRAME_TTL";
            overlay["path_remaining_ms"] = params->pathOverride.remainingMs();
            overlay["path_speed_limit"] = params->pathOverride.speedLimit;
            overlay["perception_left_count"] = params->track->pointsEdgeLeft.size();
            overlay["perception_right_count"] = params->track->pointsEdgeRight.size();
            overlay["planned_left_count"] = params->pathOverride.active()
                ? params->pathOverride.leftEdge.size() : 0;
            overlay["planned_right_count"] = params->pathOverride.active()
                ? params->pathOverride.rightEdge.size() : 0;
            overlay["planned_center_count"] = params->pathOverride.active()
                ? params->pathOverride.centerLine.size() : 0;
            overlay["lane_safety_stop"] = params->laneSafetyStop;
            overlay["planner_safety_stop"] = params->plannerSafety.latched;
            overlay["planner_rejected_source"] =
                pathSourceName(params->plannerSafety.rejectedSource);
            overlay["planner_recovery_frames"] =
                params->plannerSafety.validRecoveryFrames;
            overlay["control_geometry_source"] =
                controlGeometrySourceName(center->geometry.source);
            overlay["lanes_valid"] = lanesValid;
            overlay["lanes_frame_id"] = lanesValid ? frame.frameId : 0;
            overlay["center_valid"] = centerValid;
            overlay["center_frame_id"] = centerValid ? frame.frameId : 0;
            overlay["edge"] = {
                {"left_count", leftOverlayValid ? params->track->pointsEdgeLeft.size() : 0},
                {"right_count", rightOverlayValid ? params->track->pointsEdgeRight.size() : 0},
                {"valid_left", leftOverlayValid ? params->track->validRowsLeft : 0},
                {"valid_right", rightOverlayValid ? params->track->validRowsRight : 0},
                {"sigma_center", centerValid ? center->sigmaCenter : 0.0},
                {"line_area", centerValid ? params->ctrl.lineArea : 0},
                {"near_center", center->nearCenterValid ? center->nearCenter : -1},
                {"far_center", center->farCenterValid ? center->farCenter : -1},
                {"near_error", center->nearCenterValid
                    ? center->nearCenter - COLSIMAGE / 2 : 0},
                {"far_error", center->farCenterValid
                    ? center->farCenter - COLSIMAGE / 2 : 0},
                {"heading_error", center->headingError},
                {"heading_correction", center->headingCorrection},
                {"heading_confidence", center->headingConfidence},
                {"opposite_side_recovery", center->oppositeSideRecovery},
                {"recovery_damping", center->recoveryDamping},
                {"near_samples", center->nearCenterSamples},
                {"far_samples", center->farCenterSamples},
                {"lane_confidence", params->track->quality.confidence},
                {"common_rows", params->track->quality.commonRows},
                {"invalid_frames", center->laneInvalidFrames},
                {"recovery_frames", center->laneRecoveryFrames},
                {"unconfirmed_frames", laneUnconfirmedState.frames},
                {"left_reliable", params->track->quality.leftReliable},
                {"right_reliable", params->track->quality.rightReliable},
                {"left_strict", params->track->quality.leftReliable},
                {"right_strict", params->track->quality.rightReliable},
                {"left_single_usable", params->track->quality.leftSingleUsable},
                {"right_single_usable", params->track->quality.rightSingleUsable},
                {"left_interior_points", params->track->quality.leftInteriorPoints},
                {"right_interior_points", params->track->quality.rightInteriorPoints},
                {"left_border_ratio", params->track->quality.leftBorderRatio},
                {"right_border_ratio", params->track->quality.rightBorderRatio},
                {"lane_width_ready", center->laneWidthProfileReady()},
                {"usable_center_rows", center->usableCenterRows},
                {"weak_hybrid_active", center->recoveryMode == LaneRecoveryMode::WEAK_HYBRID},
                {"strict_dual", center->recoveryMode == LaneRecoveryMode::STRICT_DUAL},
                {"relaxed_dual", center->recoveryMode == LaneRecoveryMode::RELAXED_DUAL},
                {"selected_recovery_side", center->singleSide},
                {"single_side", center->singleSide},
                {"raw_center_jump", center->rawCenterJump},
                {"applied_center_step", center->appliedCenterStep},
                {"control_valid", center->controlValid}
            };

            auto samplePoints = [](const std::vector<PointX> &points) {
                nlohmann::json sampled = nlohmann::json::array();
                constexpr size_t stride = 4;
                for (size_t i = 0; i < points.size(); i += stride)
                    sampled.push_back({points[i].y, points[i].x});
                if (!points.empty() && (points.size() - 1) % stride != 0)
                    sampled.push_back({points.back().y, points.back().x});
                return sampled;
            };
            overlay["left"] = leftOverlayValid
                ? samplePoints(params->track->pointsEdgeLeft)
                : nlohmann::json::array();
            overlay["right"] = rightOverlayValid
                ? samplePoints(params->track->pointsEdgeRight)
                : nlohmann::json::array();
            overlay["center_line"] = centerValid
                ? samplePoints(params->ctrl.centerEdge)
                : nlohmann::json::array();
            overlay["detections"] = nlohmann::json::array();
            overlay["detections_frame_id"] = activeResultsFrameId;
            for (const auto &result : params->results)
            {
                overlay["detections"].push_back({
                    {"type", result.type}, {"label", result.label},
                    {"score", result.score}, {"x", result.x}, {"y", result.y},
                    {"w", result.width}, {"h", result.height}
                });
            }
            fsmFactory.manual->sendOverlay(overlay.dump());
        }
}
