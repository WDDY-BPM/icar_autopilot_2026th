#include "icar.hpp"

void Icar::publishTelemetry(const FrameCycle &frame)
{
        // Publish the final command after automatic/manual limiting and all
        // emergency/startup overrides, so telemetry is not one frame stale.
        fsmFactory.manual->updateVehicleState(
            params->ctrl.speed, params->ctrl.servo, params->manualTakeover);
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
            const auto recoveryModeName = [](LaneRecoveryMode mode) {
                switch (mode) {
                case LaneRecoveryMode::STRICT_DUAL: return "STRICT_DUAL";
                case LaneRecoveryMode::RELAXED_DUAL: return "RELAXED_DUAL";
                case LaneRecoveryMode::WEAK_HYBRID: return "WEAK_HYBRID";
                case LaneRecoveryMode::LEFT_SINGLE: return "LEFT_SINGLE";
                case LaneRecoveryMode::RIGHT_SINGLE: return "RIGHT_SINGLE";
                default: return "INVALID";
                }
            };
            overlay["recovery_mode"] = recoveryModeName(center->recoveryMode);
            overlay["ctrl_stop"] = params->ctrl.stop;
            overlay["must_stop"] = params->mustStop();
            overlay["stop_reasons"] = params->stopReasonString();
            overlay["ai_stale"] = params->hasStopReason(
                control_algorithms::StopReason::AI_STALE);
            overlay["camera_stop"] = params->hasStopReason(
                control_algorithms::StopReason::CAMERA);
            overlay["camera_ready"] = frame.cameraReady;
            overlay["camera_timed_out"] = frame.cameraTimedOut;
            overlay["path_override_active"] = params->pathOverride.active;
            overlay["path_source"] = pathSourceName(params->pathOverride.source);
            overlay["perception_left_count"] = params->track->pointsEdgeLeft.size();
            overlay["perception_right_count"] = params->track->pointsEdgeRight.size();
            overlay["planned_left_count"] = params->pathOverride.active
                ? params->pathOverride.leftEdge.size() : 0;
            overlay["planned_right_count"] = params->pathOverride.active
                ? params->pathOverride.rightEdge.size() : 0;
            overlay["lane_safety_stop"] = params->laneSafetyStop;
            overlay["lanes_valid"] = lanesValid;
            overlay["lanes_frame_id"] = lanesValid ? frame.frameId : 0;
            overlay["center_valid"] = centerValid;
            overlay["center_frame_id"] = centerValid ? frame.frameId : 0;
            overlay["edge"] = {
                {"left_count", leftOverlayValid ? params->track->pointsEdgeLeft.size() : 0},
                {"right_count", rightOverlayValid ? params->track->pointsEdgeRight.size() : 0},
                {"valid_left", leftOverlayValid ? center->validRowsLeft : 0},
                {"valid_right", rightOverlayValid ? center->validRowsRight : 0},
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
                {"selected_recovery_side", center->selectedRecoverySide},
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
