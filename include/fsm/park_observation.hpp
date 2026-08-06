#pragma once

#include <vector>
#include "vision/predict_result.hpp"

struct ParkObservation
{
    std::vector<PredictResult> parkMarkers;
    std::vector<PredictResult> forkMarkers;
    PredictResult gate{};
    PredictResult leftMarker{};
    PredictResult bestChoice{};
    PredictResult bestLeft{};
    bool hasGate{false};
    bool hasLeft{false};
    bool hasChoice{false};
};

ParkObservation scanParkObservation(const std::vector<PredictResult> &results);
std::vector<PredictResult> selectParkStations(
    const std::vector<PredictResult> &results, int clusterDistance = 30);
