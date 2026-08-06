#include "fsm/park/park_observation.hpp"
#include <limits>
#include <algorithm>
#include <cmath>

ParkObservation scanParkObservation(const std::vector<PredictResult> &results)
{
    ParkObservation observation;
    float bestLeftScore = -std::numeric_limits<float>::infinity();
    for (const auto &result : results)
    {
        switch (result.type)
        {
        case LABEL_PARK: observation.parkMarkers.push_back(result); break;
        case LABEL_FORK:
            if (result.width < 100 && result.height < 120 && result.y > 15)
                observation.forkMarkers.push_back(result);
            break;
        case LABEL_GATE:
            if (!observation.hasGate || result.y + result.height >
                observation.gate.y + observation.gate.height)
                observation.gate = result;
            observation.hasGate = true;
            break;
        case LABEL_CHOICE:
            if (result.width < 120 && result.height < 120 &&
                (!observation.hasChoice || result.y > observation.bestChoice.y))
            {
                observation.bestChoice = result;
                observation.hasChoice = true;
            }
            break;
        case LABEL_LEFT:
            if (result.width < 100 && result.height < 120 &&
                result.score > bestLeftScore)
            {
                bestLeftScore = result.score;
                observation.bestLeft = result;
                observation.hasLeft = true;
            }
            break;
        default: break;
        }
    }
    return observation;
}

std::vector<PredictResult> selectParkStations(
    const std::vector<PredictResult> &results, int clusterDistance)
{
    std::vector<PredictResult> candidates;
    for (const auto &result : results)
        if (result.type == LABEL_FORK && result.width < 100 &&
            result.height < 120 && result.y > 15)
            candidates.push_back(result);
    const auto centerRow = [](const PredictResult &result) {
        return result.y + result.height / 2;
    };
    std::stable_sort(candidates.begin(), candidates.end(),
        [&](const PredictResult &left, const PredictResult &right) {
            return centerRow(left) < centerRow(right);
        });
    std::vector<PredictResult> clusters;
    for (const auto &candidate : candidates)
    {
        auto match = std::find_if(clusters.begin(), clusters.end(),
            [&](const PredictResult &existing) {
                return std::abs(centerRow(existing) - centerRow(candidate)) <=
                    clusterDistance;
            });
        if (match == clusters.end())
            clusters.push_back(candidate);
        else if (candidate.score > match->score)
            *match = candidate;
    }
    std::stable_sort(clusters.begin(), clusters.end(),
        [&](const PredictResult &left, const PredictResult &right) {
            return centerRow(left) < centerRow(right);
        });
    if (clusters.size() > 2)
        clusters.resize(2);
    return clusters;
}
